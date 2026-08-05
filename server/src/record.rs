//! Wire codec for the external-sort payload (TILER.md §sort / feature_io).
//!
//! Each clipped feature becomes a sort record: the 64-bit key (tile id, layer,
//! rank) is held by the sorter, and this module encodes/decodes the value
//! payload — the feature's id, geometry (as WKB), and properties. The layer is
//! recovered from the key, so it isn't stored here.

use geo_types::Geometry;

use crate::scene::CorridorId;
use crate::synth::Synth;
use crate::tile_build::EncoderFeature;
use crate::value::Value;
use crate::wkb::{self, WkbError};

/// Why a record could not be decoded.
#[derive(Debug, PartialEq)]
pub enum RecordError {
    Truncated,
    BadValueType(u8),
    BadUtf8,
    BadSynth(u8),
    Wkb(WkbError),
}

impl From<WkbError> for RecordError {
    fn from(e: WkbError) -> Self {
        RecordError::Wkb(e)
    }
}

impl std::fmt::Display for RecordError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecordError::Truncated => write!(f, "record truncated"),
            RecordError::BadValueType(t) => write!(f, "bad value type {t}"),
            RecordError::BadUtf8 => write!(f, "invalid utf-8 in record"),
            RecordError::BadSynth(t) => write!(f, "bad synth tag {t}"),
            RecordError::Wkb(e) => write!(f, "wkb error: {e:?}"),
        }
    }
}
impl std::error::Error for RecordError {}

/// Encodes a feature into a sort-record payload.
pub fn encode(feature: &EncoderFeature) -> Vec<u8> {
    RecordEncoder::new(feature.id, &feature.properties, feature.synth).encode(&feature.geometry)
}

/// Encodes the sort records of one source feature. A feature fans out to one
/// record per (zoom, tile) it covers — thousands for large geometries — so the
/// id, property block, and synth tag are serialized once here and only the
/// geometry is encoded per record.
pub struct RecordEncoder {
    id_bytes: [u8; 8],
    /// Serialized property count + entries, shared verbatim by every record.
    props: Vec<u8>,
    /// Serialized [`Synth`] tag: kind byte + two u32 parameters.
    synth: [u8; 9],
}

impl RecordEncoder {
    pub fn new(id: u64, properties: &[(String, Value)], synth: Synth) -> Self {
        let mut props = Vec::new();
        props.extend_from_slice(&(properties.len() as u32).to_le_bytes());
        for (key, value) in properties {
            write_bytes(&mut props, key.as_bytes());
            write_value(&mut props, value);
        }
        RecordEncoder { id_bytes: id.to_le_bytes(), props, synth: encode_synth(synth) }
    }

    /// Encodes one record payload for a clipped geometry.
    pub fn encode(&self, geometry: &Geometry) -> Vec<u8> {
        let geom = wkb::to_wkb(geometry);
        let mut buf = Vec::with_capacity(8 + 4 + geom.len() + self.props.len() + 9);
        buf.extend_from_slice(&self.id_bytes);
        buf.extend_from_slice(&(geom.len() as u32).to_le_bytes());
        buf.extend_from_slice(&geom);
        buf.extend_from_slice(&self.props);
        buf.extend_from_slice(&self.synth);
        buf
    }
}

/// Decodes a sort-record payload back into a feature.
pub fn decode(data: &[u8]) -> Result<EncoderFeature, RecordError> {
    let mut cur = Reader { data, pos: 0 };
    let id = cur.u64()?;

    let geom_len = cur.u32()? as usize;
    let geom_bytes = cur.take(geom_len)?;
    let geometry: Geometry = wkb::parse(geom_bytes)?;

    let nprops = cur.u32()? as usize;
    let mut properties = Vec::with_capacity(nprops);
    for _ in 0..nprops {
        let key = cur.string()?;
        let value = cur.value()?;
        properties.push((key, value));
    }
    let synth = cur.synth()?;
    Ok(EncoderFeature { id, geometry, properties, elevation: None, z: None, mesh: None, synth })
}

/// Sentinel for "no corridor" in a [`Synth::Road`] parameter slot.
const NO_CORRIDOR: u32 = u32::MAX;

/// Serializes a [`Synth`] as a fixed kind byte + two u32 parameters.
fn encode_synth(synth: Synth) -> [u8; 9] {
    let (tag, a, b): (u8, u32, u32) = match synth {
        Synth::None => (0, 0, 0),
        Synth::Road { corridor, deck } => (1, corridor.unwrap_or(NO_CORRIDOR), deck as u32),
        Synth::Structure { corridor, kind } => {
            (2, corridor, matches!(kind, crate::scene::SpanKind::Tunnel) as u32)
        }
        Synth::DrapedDeck => (3, 0, 0),
    };
    let mut out = [0u8; 9];
    out[0] = tag;
    out[1..5].copy_from_slice(&a.to_le_bytes());
    out[5..9].copy_from_slice(&b.to_le_bytes());
    out
}

fn write_bytes(buf: &mut Vec<u8>, bytes: &[u8]) {
    buf.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
    buf.extend_from_slice(bytes);
}

fn write_value(buf: &mut Vec<u8>, value: &Value) {
    match value {
        Value::String(s) => {
            buf.push(0);
            write_bytes(buf, s.as_bytes());
        }
        Value::Int(i) => {
            buf.push(1);
            buf.extend_from_slice(&i.to_le_bytes());
        }
        Value::Double(d) => {
            buf.push(2);
            buf.extend_from_slice(&d.to_le_bytes());
        }
        Value::Bool(b) => {
            buf.push(3);
            buf.push(*b as u8);
        }
    }
}

struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], RecordError> {
        let end = self.pos.checked_add(n).ok_or(RecordError::Truncated)?;
        let slice = self.data.get(self.pos..end).ok_or(RecordError::Truncated)?;
        self.pos = end;
        Ok(slice)
    }

    fn u32(&mut self) -> Result<u32, RecordError> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }

    fn u64(&mut self) -> Result<u64, RecordError> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }

    fn i64(&mut self) -> Result<i64, RecordError> {
        Ok(i64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }

    fn f64(&mut self) -> Result<f64, RecordError> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }

    fn string(&mut self) -> Result<String, RecordError> {
        let len = self.u32()? as usize;
        let bytes = self.take(len)?;
        String::from_utf8(bytes.to_vec()).map_err(|_| RecordError::BadUtf8)
    }

    fn value(&mut self) -> Result<Value, RecordError> {
        match self.take(1)?[0] {
            0 => Ok(Value::String(self.string()?)),
            1 => Ok(Value::Int(self.i64()?)),
            2 => Ok(Value::Double(self.f64()?)),
            3 => Ok(Value::Bool(self.take(1)?[0] != 0)),
            other => Err(RecordError::BadValueType(other)),
        }
    }

    fn synth(&mut self) -> Result<Synth, RecordError> {
        let tag = self.take(1)?[0];
        let a = self.u32()?;
        let b = self.u32()?;
        match tag {
            0 => Ok(Synth::None),
            1 => Ok(Synth::Road {
                corridor: (a != NO_CORRIDOR).then_some(a as CorridorId),
                deck: b == 1,
            }),
            2 => Ok(Synth::Structure {
                corridor: a as CorridorId,
                kind: if b == 1 {
                    crate::scene::SpanKind::Tunnel
                } else {
                    crate::scene::SpanKind::Bridge
                },
            }),
            3 => Ok(Synth::DrapedDeck),
            other => Err(RecordError::BadSynth(other)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{Coord, LineString, Point};

    #[test]
    fn roundtrips_feature() {
        let f = EncoderFeature {
            id: 0xDEAD_BEEF,
            geometry: Geometry::LineString(LineString(vec![
                Coord { x: 1.0, y: 2.0 },
                Coord { x: 3.0, y: 4.0 },
            ])),
            properties: vec![
                ("class".to_string(), Value::String("primary".into())),
                ("rank".to_string(), Value::Int(3)),
                ("height".to_string(), Value::Double(12.5)),
                ("bridge".to_string(), Value::Bool(true)),
            ],
            elevation: None, z: None, mesh: None, synth: Synth::None,
        };
        let decoded = decode(&encode(&f)).unwrap();
        assert_eq!(decoded.id, f.id);
        assert_eq!(decoded.geometry, f.geometry);
        assert_eq!(decoded.properties, f.properties);
    }

    #[test]
    fn roundtrips_point_without_properties() {
        let f = EncoderFeature {
            id: 0,
            geometry: Geometry::Point(Point::new(7.0, 8.0)),
            properties: vec![],
            elevation: None, z: None, mesh: None, synth: Synth::None,
        };
        let decoded = decode(&encode(&f)).unwrap();
        assert_eq!(decoded.geometry, f.geometry);
        assert!(decoded.properties.is_empty());
    }

    #[test]
    fn roundtrips_synth_tags() {
        use crate::scene::SpanKind;
        for synth in [
            Synth::None,
            Synth::Road { corridor: None, deck: false },
            Synth::Road { corridor: Some(7), deck: true },
            Synth::Structure { corridor: 3, kind: SpanKind::Bridge },
            Synth::Structure { corridor: 9, kind: SpanKind::Tunnel },
        ] {
            let f = EncoderFeature {
                id: 1,
                geometry: Geometry::Point(Point::new(1.0, 1.0)),
                properties: vec![],
                elevation: None, z: None, mesh: None, synth,
            };
            assert_eq!(decode(&encode(&f)).unwrap().synth, synth, "{synth:?}");
        }
    }

    #[test]
    fn truncated_payload_errors() {
        let f = EncoderFeature {
            id: 1,
            geometry: Geometry::Point(Point::new(1.0, 1.0)),
            properties: vec![],
            elevation: None, z: None, mesh: None, synth: Synth::None,
        };
        let bytes = encode(&f);
        assert!(matches!(decode(&bytes[..bytes.len() - 3]), Err(RecordError::Truncated)));
    }
}
