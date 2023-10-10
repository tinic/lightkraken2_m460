#define _CRT_SECURE_NO_WARNINGS

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

#ifdef WIN32
#include "dirent.h"
#else // #ifdef (WIN32)
#include <dirent.h>
#endif  // #ifdef (WIN32)

#include "fx_api.h"

extern void _fx_ram_driver(FX_MEDIA *media_ptr);
static unsigned char media_memory[512] = { 0 };
static FX_MEDIA ram_disk = { 0 };

void fwriteAsCHeader(FILE *fp, unsigned char arr[], size_t length) {
    fprintf(fp, "unsigned char fs_data[%d] = {\n/* %08x */ ", (int)length, (unsigned int)(0));
    
    for (size_t i = 0; i < length; ++i) {
        fprintf(fp, "0x%02X", arr[i]);
        
        // If not the last element, print a comma.
        if (i != length - 1) {
            fprintf(fp, ", ");
        }

        if (i % 16 == 15 && i != length - 1) {
            fprintf(fp, "/* ");
            for (size_t j = i - 15; j <= i && j < length; j++) {
                unsigned char ch = arr[j];
                fprintf(fp, "%c", isprint(ch) ? ch : '.');
            }
            fprintf(fp, " */");
        }

        // Optional: format output for better readability by breaking lines.
        if ((i + 1) % 16 == 0 && i != length - 1) {
            fprintf(fp, "\n/* %08x */ ", (unsigned int)(i + 1));
        }

    }

    fprintf(fp, "\n};\n");
}

static int should_strip(char c) {
    // Check for slash, backspace or whitespace
    return c == '/' || c == '\\' || c == '\b' || c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

void strip_path(const char *path, char *result) {
    size_t end = strlen(path) - 1;

    // Find the end index
    while(should_strip(path[end]) && end > 0) {
        end--;
    }

    // Copy the substring to the result
    int j = 0;
    for(int i = 0; i <= end; i++) {
        result[j++] = path[i];
    }
    result[j] = '\0';  // Null-terminate the result string
}

static void copy_tree(FX_MEDIA *media, const char *in_full_path, const char *in_local_path) {

    DIR *dir = opendir(in_full_path);
    if (!dir) {
        printf("read_directory failed!\n");
        exit(-1);
    }

    struct dirent *entry = 0;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue; // skip current and parent directories
        }

        char full_path[1024] = { 0 };
        snprintf(full_path, sizeof(full_path), "%s/%s", in_full_path, entry->d_name);
        char local_path[1024] = { 0 };
        snprintf(local_path, sizeof(local_path), "%s/%s", in_local_path, entry->d_name);

        // Check if the entry is a directory or file
        struct stat statbuf = { 0 };
        if (stat(full_path, &statbuf) == -1) {
            continue;
        }

        if (S_ISDIR(statbuf.st_mode)) {
            fx_directory_create(media, local_path);
            fx_directory_default_set(media, local_path);

            copy_tree(media, full_path, local_path);
        } else {

            FILE *file = fopen(full_path, "rb");
            if (!file) {
                printf("fopen failed!\n");
                exit(-1);
            }

            fseek(file, 0, SEEK_END);
            size_t fileSize = ftell(file);
            fseek(file, 0, SEEK_SET);

            char *buffer = (char *)malloc(fileSize + 1);
            if (!buffer) {
                printf("Failed to allocate memory!\n");
                exit(-1);
            }

            size_t bytesRead = fread(buffer, 1, fileSize, file);
            if (bytesRead != fileSize) {
                printf("Failed to read file\n");
                exit(-1);
            }

            fclose(file);

            UINT status = 0;

            if ( (status = fx_file_create(media, local_path)) != FX_SUCCESS) {
                printf("fx_file_create failed!\n");
                exit(-1);
            }

            FX_FILE fx_file = { 0 };
            if ( (status = fx_file_open(media, &fx_file, local_path, FX_OPEN_FOR_WRITE)) != FX_SUCCESS) {
                printf("fx_file_open failed!\n");
                exit(-1);
            }

            if ( (status = fx_file_allocate(&fx_file, (ULONG)bytesRead) != FX_SUCCESS)) {
                printf("_fx_file_allocate failed!\n");
                exit(-1);
            }
            
            if ( (status = fx_file_write(&fx_file, buffer, (ULONG)bytesRead) != FX_SUCCESS)) {
                printf("fx_file_write failed!\n");
                exit(-1);
            }

            fx_file_close(&fx_file);

            free(buffer);
        }
    }

    closedir(dir);
}

void print_contents(FX_MEDIA *media, CHAR *default_dir, ULONG *total_bytes, ULONG *sector_bytes) {
    CHAR entry[64] = { 0 };
    CHAR dirs[64][64] = { 0 };
    UINT dir_count = 0;
    UINT attributes = 0;
    ULONG size = 0;
    if (fx_directory_default_set(&ram_disk, default_dir) != FX_SUCCESS) {
        return;
    }
    if (fx_directory_first_full_entry_find(media, entry, &attributes, &size, NULL, NULL, NULL, NULL, NULL, NULL) == FX_SUCCESS) {
        do {
            if (strcmp(entry,".") == 0 ||
                strcmp(entry,"..") == 0) {
                continue;
            }
            if ((attributes & FX_DIRECTORY)) {
                strcpy(dirs[dir_count++], entry);
            } else {
                printf("file<%s/%s> size<%d bytes>\n", default_dir ? default_dir : "", entry, (int)size);
                *total_bytes += size;
                *sector_bytes += (size + media->fx_media_bytes_per_sector) & (~(media->fx_media_bytes_per_sector-1));
            }
        } while (fx_directory_next_full_entry_find(media, entry, &attributes, &size, NULL, NULL, NULL, NULL, NULL, NULL) == FX_SUCCESS);
    }
    for (UINT c = 0; c < dir_count; c++) {
        CHAR local_path[1024] = { 0 };
        sprintf(local_path, "%s/%s", default_dir ? default_dir : "", dirs[c]);
        print_contents(media, local_path, total_bytes, sector_bytes);
    }
}

int strToBool(const char* str) {
    if (strcasecmp(str, "YES") == 0 || 
        strcasecmp(str, "TRUE") == 0 || 
        strcasecmp(str, "1") == 0) {
        return 1;
    }
    return 0;
}

int main(int argc, char *argv[]) {


    if (argc != 6) {
        fprintf(stderr, "Usage: %s <source_path> <num_sectors> <sector_size> <header_file> <image_file>\n", argv[0]);
        return 1;  // exit with error code
    }

    clock_t start, end;
    double cpu_time_used;

#ifdef WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif  // #ifdef WIN32

    start = clock();  // Get the start time

    printf("🍳🥓mkfat generating header and image file from source directory...\n");

    const char *source_path = argv[1];
    const size_t num_sectors = atoi(argv[2]);
    const size_t sector_size = atoi(argv[3]);
    const char *header_file = argv[4];
    const char *image_file = argv[5];

    if (num_sectors <= 0) {
        printf("Invalid num_sectors!\n");
        exit(-1);
    }

    fx_system_initialize();

    UINT status;
#if 1
    const size_t fs_size = num_sectors * sector_size;
    unsigned char *fs_data = malloc(fs_size);
    memset(fs_data, 0, fs_size);
    status = fx_media_format(&ram_disk,
        _fx_ram_driver,         // Driver entry
        fs_data,                // RAM disk memory pointer
        media_memory,           // Media buffer pointer
        sizeof(media_memory),   // Media buffer size
        "LIGHTKRAKEN",          // Volume Name
        1,                      // Number of FATs
        32,                     // Directory Entries
        0,                      // Hidden sectors
        (ULONG)num_sectors,     // Total sectors
        (ULONG)sector_size,     // Sector size
        1,                      // Sectors per cluster
        1,                      // Heads
        1);                     // Sectors per track
    if (status) {
        perror("fx_media_format failed!\n");
        return -1;
    }
#else  // #if 1
    const size_t fs_size = num_sectors * sector_size;
    unsigned char *fs_data = malloc(fs_size);
    memset(fs_data, 0, fs_size);
    status = fx_media_exFAT_format(&ram_disk,
        _fx_ram_driver,         // Driver entry
        fs_data,                // RAM disk memory pointer
        media_memory,           // Media buffer pointer
        sizeof(media_memory),   // Media buffer size
        "LIGHTKRAKEN2",         // Volume Name
        1,                      // Number of FATs
        0,                      // Hidden sectors
        (ULONG)num_sectors,     // Total sectors
        (ULONG)sector_size,     // Sector size
        8,                      // exFAT Sectors per cluster
        12345,                  // Volume ID
        1);                     // Boundary unit
    if (status) {
        perror("fx_media_exFAT_format failed!\n");
        return -1;
    }
#endif  // #if 1

    status = fx_media_open(&ram_disk, 
        "RAM Disk", 
        _fx_ram_driver, 
        fs_data, 
        media_memory, 
        sizeof(media_memory));
    if (status) {
        printf("fx_media_open failed!\n");
        return 0;
    }

    printf("total image size is %d bytes\n", (int)(ram_disk.fx_media_bytes_per_sector * ram_disk.fx_media_total_sectors));

    fx_directory_default_set(&ram_disk, NULL);

    char stripped_source_path[1024];
    strip_path(source_path, stripped_source_path);
    copy_tree(&ram_disk, stripped_source_path, "");

    ULONG total_bytes = 0;
    ULONG sector_bytes = 0;
    print_contents(&ram_disk, NULL, &total_bytes, &sector_bytes);
    printf("total bytes on disk: %d bytes (%d actual sector bytes)\n", (int)total_bytes, (int)sector_bytes);

    fx_media_close(&ram_disk);

    FILE *fp_header = fopen(header_file, "wb");
    if (fp_header) {
        fwriteAsCHeader(fp_header, fs_data, num_sectors * sector_size);
        fclose(fp_header);
    }

    FILE *fp_img = fopen(image_file, "wb");
    if (fp_img) {
        fwrite(fs_data, 1, num_sectors * sector_size, fp_img);
        fclose(fp_img);
    }

    end = clock();  // Get the end time

    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;  // Calculate the elapsed time in seconds

    printf("🍳🥓mkfat done in %.2f seconds.\n", cpu_time_used);

    return 0;
}
