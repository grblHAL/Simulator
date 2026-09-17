/*
  fs_posix.h - VFS wrapper/mount for host file system (directory)

  Part of grblHAL

  Copyright (c) 2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if FS_ENABLE & FS_POSIX

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <stdio.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/platform.h"
#include "grbl/vfs.h"

static char _cwd[51] = "/";
static vfs_path_t cwd = { .name = _cwd, .len = sizeof(_cwd) - 1 }, root = {0};

FLASHMEM static const char *get_path (const char *path)
{
    static vfs_path_t abspath = {0};

    if(root.len + strlen(cwd.name) + strlen(path) + 1 > abspath.len) {
        abspath.len = max(50, root.len + strlen(cwd.name)) + strlen(path) + 1;
        abspath.name = realloc(abspath.name, abspath.len);
    }

    if(abspath.name) {

        vfs_fixpath(strcat(strcpy(abspath.name, root.name), *path == '/' ? "" : cwd.name));

        if(*path && strcmp(path, "/")) {

            char *newpath;

            if((newpath = malloc(strlen(path) + 1))) {

                strcpy(newpath, path);

                char *p, *el = strtok(newpath, "/");

                while(el) {
                    if(!strcmp("..", el)) {
                        if((p = strrchr(abspath.name, '/')))
                            *(p + (p == abspath.name ? 1 : 0)) = '\0';
                    } else if(*el && strcmp(el, "."))
                        strcat(strcat(abspath.name, "/"), el);
                    el = strtok(NULL, "/");
                }

                free(newpath);
            } else
                strcat(abspath.name, path);
        }
    } else
        abspath.len = 0;

//printf("Path: %s\n", abspath.name ? (const char *)abspath.name : path);

    return abspath.name ? (const char *)abspath.name : path;
}

FLASHMEM static vfs_file_t *fs_open (const char *filename, const char *mode)
{
    FILE *f;
    vfs_file_t *file = malloc(sizeof(vfs_file_t));

    if(file) {

        if((f = fopen((filename = get_path(filename)), mode))) {
            struct stat st;
            if(stat(filename, &st) == 0)
                file->size = st.st_size;
            file->handle = f;
        } else {
            free(file);
            file = NULL;
        }
    }

    return file;
}

FLASHMEM static void fs_close (vfs_file_t *file)
{
    fclose((FILE *)file->handle);
    free(file);
}

static size_t fs_read (void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return fread(buffer, size, count, (FILE *)file->handle);
}

static size_t fs_write (const void *buffer, size_t size, size_t count, vfs_file_t *file)
{
    return fwrite(buffer, size, count, (FILE *)file->handle);
}

FLASHMEM static size_t fs_tell (vfs_file_t *file)
{
    return ftell((FILE *)file->handle);
}

FLASHMEM static int fs_seek (vfs_file_t *file, size_t offset)
{
    return fseek((FILE *)file->handle, offset, SEEK_SET);
}

FLASHMEM static int fs_truncate (vfs_file_t *file, size_t length)
{
    return ftruncate(fileno((FILE *)file->handle), length);
}

FLASHMEM static bool fs_eof (vfs_file_t *file)
{
    return feof((FILE *)file->handle) != 0;
}

FLASHMEM static int fs_rename (const char *from, const char *to)
{
	int ret;
	char *old;
	const char *f = get_path(from);

	if((old = malloc(strlen(f) + 1))) {
		strcpy(old, f);
		ret = rename(old, get_path(to));
		free(old);
	} else
		ret = ENOMEM;
	
	return ret;
}

FLASHMEM static int fs_unlink (const char *filename)
{
    return unlink(get_path(filename));
}

FLASHMEM static int fs_stat (const char *filename, vfs_stat_t *st)
{
    struct stat f;

    if((vfs_errno = stat(get_path(filename), &f)) == 0) {
        st->st_size = f.st_size;
        if(!(st->st_mode.directory = (f.st_mode & S_IFMT) == S_IFDIR))
            st->st_mode.system = (f.st_mode & S_IFMT) != S_IFREG;
		//st->st_mtime = f.st_mtime;
    }

    return vfs_errno ? -1 : 0;
}

FLASHMEM static int fs_mkdir (const char *path)
{
#ifdef _WIN32
    return mkdir(get_path(path));
#else
    return mkdir(get_path(path), 0755);
#endif
}

FLASHMEM static int fs_chdir (const char *path)
{
    int ferrno;
    vfs_stat_t st;

    if((ferrno = fs_stat(path, &st)) == 0 && st.st_mode.directory) {
        size_t cwdlen;
        if((cwdlen = strlen(path)) > cwd.len) {
            if(cwd.name == _cwd)
                cwd.name = malloc(cwdlen + 1);
            else
                cwd.name = realloc(cwd.name, cwdlen + 1);
            if(cwd.name)
                cwd.len = cwdlen;
            else {
                cwd.name = _cwd;
                cwd.len = sizeof(_cwd) - 1;
                path = "/";
                ferrno = -1;
            }
        }
        strcpy(cwd.name, *path ? path : "/");
    } else
        ferrno = ENOTDIR;

    return ferrno;
}

FLASHMEM static int fs_rmdir (const char *path)
{
    return rmdir(get_path(path));
}

FLASHMEM static char *fs_getcwd (char *buf, size_t size)
{
    return cwd.name;
}

FLASHMEM static vfs_dir_t *fs_opendir (const char *path)
{
    DIR *d;
    vfs_dir_t *dir = malloc(sizeof(vfs_dir_t));

    if((d = opendir(get_path(path))))
        dir->handle = d;
    else {
        free(dir);
        dir = NULL;
    }

    return dir;
}

FLASHMEM static void fs_closedir (vfs_dir_t *dir)
{
    if (dir) {
        vfs_errno = closedir((DIR *)dir->handle);
        free(dir);
    }
}

FLASHMEM static char *fs_readdir (vfs_dir_t *dir, vfs_dirent_t *dirent)
{
    struct dirent *de;
    vfs_stat_t st;

    memset(dirent, 0, sizeof(vfs_dirent_t));

    if(!(de = readdir((DIR *)dir->handle)))
        return NULL;

    while(!strcmp(de->d_name, ".") || !strcmp(de->d_name, "..")) {
        if(!(de = readdir((DIR *)dir->handle)))
            return NULL;
    }

    strcpy(dirent->name, de->d_name);

    if((vfs_errno = fs_stat(dirent->name, &st)) == 0) {
        dirent->size = st.st_size;
        dirent->st_mode.mode = st.st_mode.mode;
    }

    return dirent->name;
}

FLASHMEM static int fs_chmod (const char *filename, vfs_st_mode_t attr, vfs_st_mode_t mask)
{
    return -1; //(vfs_errno = f_chmod(filename, attr.mode, mask.mode)) == FR_OK ? 0 : -1;
}

FLASHMEM static int fs_utime (const char *filename, struct tm *modified)
{
/*
    FILINFO fno;

    fno.fdate = (WORD)(((modified->tm_year - 80) * 512U) | (modified->tm_mon + 1) * 32U | modified->tm_mday);
    fno.ftime = (WORD)(modified->tm_hour * 2048U | modified->tm_min * 32U | modified->tm_sec / 2U);

    return f_utime(filename, &fno);
*/
    return -1;
}

FLASHMEM static bool fs_getfree (vfs_free_t *free)
{
    return false;
/*
    FATFS *fs;
    DWORD fre_clust;
    uint64_t tot_sect;

    if((vfs_errno = f_getfree("", &fre_clust, &fs)) == FR_OK) {
        tot_sect = (fs->n_fatent - 2) * fs->csize;
        free->size = tot_sect << 9; // assuming 512 byte sector size
        free->used = (tot_sect - fre_clust * fs->csize) << 9;
    }

    return vfs_errno == FR_OK;
*/
}

FLASHMEM bool fs_posix_mount (const char *path, const char *dir)
{
    PROGMEM static const vfs_t fs = {
        .fs_name = "PosixFs",
        .removable = true,
        .fopen = fs_open,
        .fclose = fs_close,
        .fread = fs_read,
        .fwrite = fs_write,
        .ftell = fs_tell,
        .fseek = fs_seek,
        .ftruncate = fs_truncate,
        .feof = fs_eof,
        .frename = fs_rename,
        .funlink = fs_unlink,
        .fmkdir = fs_mkdir,
        .fchdir = fs_chdir,
        .frmdir = fs_rmdir,
        .fopendir = fs_opendir,
        .readdir = fs_readdir,
        .fclosedir = fs_closedir,
        .fchmod = fs_chmod,
        .fstat = fs_stat,
        .futime = fs_utime,
        .fgetcwd = fs_getcwd,
        .fgetfree = fs_getfree
    };

    struct stat st;

    if(stat(dir, &st) == 0 && (st.st_mode & S_IFMT) == S_IFDIR) {
        root.len = strlen(dir) + 1;                                         
        if((root.name = malloc(root.len))) {
            vfs_fixpath(strcpy(root.name, dir));
            if(!vfs_mount(NULL, path, &fs, (vfs_st_mode_t){0})) {
                free(root.name);
                root.name = NULL;
            }
        }
    }

    return !!root.name;
}

#endif // FS_ENABLE & FS_POSIX
