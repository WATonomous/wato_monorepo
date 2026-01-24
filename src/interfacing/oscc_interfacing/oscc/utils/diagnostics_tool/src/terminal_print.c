// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file terminal_print.c
 * @brief Debug printer functions.
 *
 */

#include "terminal_print.h"

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

// *****************************************************
// static global veriables
// *****************************************************

static char lines[MAX_LINES][LINE_SIZE];

static int num_lines = 0;

static int last_lines = 0;

// *****************************************************
// static definitions
// *****************************************************

//
static void print_new_line(const char * name)
{
  printf("%s\n", name);
}

// *****************************************************
// public definitions
// *****************************************************

//
void add_line(char * line)
{
  strcpy(lines[num_lines], line);

  num_lines++;
}

//
void print_lines()
{
  int i;

  for (i = 0; i < last_lines; i++) {
    fputs("\033[A\033[2K", stdout);

    rewind(stdout);

    ftruncate(1, 0);
  }

  last_lines = 0;

  for (i = 0; i < num_lines; i++) {
    print_new_line(lines[i]);

    last_lines++;
  }

  num_lines = 0;
}
