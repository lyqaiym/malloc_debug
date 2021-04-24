
#pragma once

#include <errno.h>
#include <inttypes.h>
#include <malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/param.h>
#include <unistd.h>

#include <mutex>
#include <vector>

//#include <android-base/file.h>
#include <android-base/stringprintf.h>
#include <private/bionic_malloc_dispatch.h>
#include <private/MallocXmlElem.h>
#include <sys/system_properties.h>

#include <android-base/unique_fd.h>
#include <fcntl.h>
#include <sys/stat.h>


bool ReadFdToString(int fd, std::string* content);

bool ReadFileToString(const std::string& path, std::string* content, bool follow_symlinks);