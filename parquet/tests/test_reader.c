/**
 * @file test_reader.c
 * @brief Tests for Parquet file reading (from carquet upstream, adapted)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <carquet/carquet.h>

#define TEST_PASS(name) printf("[PASS] %s\n", name)
#define TEST_FAIL(name, msg) do { printf("[FAIL] %s: %s\n", name, msg); return 1; } while(0)

#define TEST_FILE "/tmp/carquet_test_reader.parquet"

static int test_reader_options(void) {
    carquet_reader_options_t opts;
    carquet_reader_options_init(&opts);

    assert(opts.use_mmap == false);
    assert(opts.verify_checksums == true);
    assert(opts.buffer_size == 64 * 1024);
    assert(opts.num_threads == 0);

    TEST_PASS("reader_options");
    return 0;
}

static int test_writer_options(void) {
    carquet_writer_options_t opts;
    carquet_writer_options_init(&opts);

    assert(opts.compression == CARQUET_COMPRESSION_UNCOMPRESSED);
    assert(opts.row_group_size == 128 * 1024 * 1024);
    assert(opts.page_size == 1024 * 1024);
    assert(opts.write_statistics == true);
    assert(opts.created_by != NULL);

    TEST_PASS("writer_options");
    return 0;
}

static int test_open_nonexistent(void) {
    carquet_error_t err = CARQUET_ERROR_INIT;
    carquet_reader_t* reader = carquet_reader_open(
        "/nonexistent/path/file.parquet", NULL, &err);
    (void)reader;

    assert(reader == NULL);
    assert(err.code == CARQUET_ERROR_FILE_OPEN);

    TEST_PASS("open_nonexistent");
    return 0;
}

static int test_type_names(void) {
    assert(strcmp(carquet_physical_type_name(CARQUET_PHYSICAL_BOOLEAN), "BOOLEAN") == 0);
    assert(strcmp(carquet_physical_type_name(CARQUET_PHYSICAL_INT32), "INT32") == 0);
    assert(strcmp(carquet_physical_type_name(CARQUET_PHYSICAL_INT64), "INT64") == 0);
    assert(strcmp(carquet_physical_type_name(CARQUET_PHYSICAL_DOUBLE), "DOUBLE") == 0);
    assert(strcmp(carquet_physical_type_name(CARQUET_PHYSICAL_BYTE_ARRAY), "BYTE_ARRAY") == 0);

    assert(strcmp(carquet_compression_name(CARQUET_COMPRESSION_UNCOMPRESSED), "UNCOMPRESSED") == 0);
    assert(strcmp(carquet_compression_name(CARQUET_COMPRESSION_SNAPPY), "SNAPPY") == 0);
    assert(strcmp(carquet_compression_name(CARQUET_COMPRESSION_GZIP), "GZIP") == 0);
    assert(strcmp(carquet_compression_name(CARQUET_COMPRESSION_LZ4), "LZ4") == 0);
    assert(strcmp(carquet_compression_name(CARQUET_COMPRESSION_ZSTD), "ZSTD") == 0);

    assert(strcmp(carquet_encoding_name(CARQUET_ENCODING_PLAIN), "PLAIN") == 0);
    assert(strcmp(carquet_encoding_name(CARQUET_ENCODING_RLE), "RLE") == 0);
    assert(strcmp(carquet_encoding_name(CARQUET_ENCODING_RLE_DICTIONARY), "RLE_DICTIONARY") == 0);

    TEST_PASS("type_names");
    return 0;
}

static int test_status_strings(void) {
    assert(strcmp(carquet_status_string(CARQUET_OK), "Success") == 0);
    assert(strcmp(carquet_status_string(CARQUET_ERROR_FILE_NOT_FOUND), "File not found") == 0);
    assert(strcmp(carquet_status_string(CARQUET_ERROR_INVALID_MAGIC), "Invalid magic bytes") == 0);
    assert(strcmp(carquet_status_string(CARQUET_ERROR_OUT_OF_MEMORY), "Out of memory") == 0);

    TEST_PASS("status_strings");
    return 0;
}

static int test_write_simple_file(void) {
    carquet_error_t err = CARQUET_ERROR_INIT;

    /* Create schema */
    carquet_schema_t* schema = carquet_schema_create(&err);
    if (!schema) {
        TEST_FAIL("write_simple_file", "schema creation failed");
    }

    /* Add columns */
    carquet_status_t status = carquet_schema_add_column(
        schema, "id", CARQUET_PHYSICAL_INT32, NULL,
        CARQUET_REPETITION_REQUIRED, 0, 0);
    assert(status == CARQUET_OK);

    status = carquet_schema_add_column(
        schema, "value", CARQUET_PHYSICAL_DOUBLE, NULL,
        CARQUET_REPETITION_REQUIRED, 0, 0);
    assert(status == CARQUET_OK);

    /* Create writer */
    carquet_writer_options_t opts;
    carquet_writer_options_init(&opts);
    opts.compression = CARQUET_COMPRESSION_UNCOMPRESSED;

    carquet_writer_t* writer = carquet_writer_create(TEST_FILE, schema, &opts, &err);
    if (!writer) {
        carquet_schema_free(schema);
        TEST_FAIL("write_simple_file", "writer creation failed");
    }

    /* Write some data */
    const int num_rows = 100;
    int32_t ids[100];
    double values[100];

    for (int i = 0; i < num_rows; i++) {
        ids[i] = i;
        values[i] = (double)i * 1.5;
    }

    status = carquet_writer_write_batch(writer, 0, ids, num_rows, NULL, NULL);
    assert(status == CARQUET_OK);

    status = carquet_writer_write_batch(writer, 1, values, num_rows, NULL, NULL);
    assert(status == CARQUET_OK);

    /* Close writer */
    status = carquet_writer_close(writer);
    if (status != CARQUET_OK) {
        carquet_schema_free(schema);
        TEST_FAIL("write_simple_file", "writer close failed");
    }

    carquet_schema_free(schema);

    /* Verify file exists and has correct structure */
    FILE* f = fopen(TEST_FILE, "rb");
    if (!f) {
        TEST_FAIL("write_simple_file", "output file not found");
    }

    /* Check PAR1 header */
    char magic[4];
    (void)magic;
    assert(fread(magic, 1, 4, f) == 4);
    assert(memcmp(magic, "PAR1", 4) == 0);

    /* Check PAR1 footer */
    fseek(f, -4, SEEK_END);
    assert(fread(magic, 1, 4, f) == 4);
    assert(memcmp(magic, "PAR1", 4) == 0);

    fclose(f);

    /* Read the file back */
    carquet_reader_t* reader = carquet_reader_open(TEST_FILE, NULL, &err);
    if (!reader) {
        remove(TEST_FILE);
        TEST_FAIL("write_simple_file", "reader open failed");
    }

    /* Verify basic metadata */
    int64_t read_rows = carquet_reader_num_rows(reader);
    int32_t read_cols = carquet_reader_num_columns(reader);

    assert(read_rows == num_rows);
    assert(read_cols == 2);

    carquet_reader_close(reader);
    remove(TEST_FILE);

    TEST_PASS("write_simple_file");
    return 0;
}

int main(void) {
    int failures = 0;

    printf("=== Reader Tests ===\n\n");

    failures += test_reader_options();
    failures += test_writer_options();
    failures += test_open_nonexistent();
    failures += test_type_names();
    failures += test_status_strings();
    failures += test_write_simple_file();

    printf("\n");
    if (failures == 0) {
        printf("All tests passed!\n");
        return 0;
    } else {
        printf("%d test(s) failed\n", failures);
        return 1;
    }
}
