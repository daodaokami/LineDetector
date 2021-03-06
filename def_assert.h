
#pragma once

#include <cxxabi.h>
#include <unistd.h>
#include <execinfo.h> // for stack trace
#include <cstdlib>   // for abort
#include <stdio.h>
#include <stdlib.h>
#define  COLOR_RESET   "\033[0m"
#define  COLOR_RED     "\033[31m"
#define  COLOR_GREEN   "\033[32m"

#define  INFO_STREAM(x) std::cout<<"\033[32m[INFO]   "<< __FUNCTION__ << " : " <<x<<"\033[0;0m"<<std::endl;
#define  WARN_STREAM(x) std::cerr<<"\033[0;33m[WARN] "<< __FUNCTION__ << " : " <<x<<"\033[0;0m"<<std::endl;


namespace utils {

inline void assert_fail(const char *condition, const char *function,
                        const char *file, int line)
{
  fprintf(stderr,  COLOR_RED " ASSERT failed: %s in function %s at %s: %i\n"  COLOR_RESET,
          condition, function, file, line);

  fprintf(stderr,  COLOR_RED "Stacktrace:\n"  COLOR_RESET);

  // Get and print stack trace with demangled names. Taken from:
  // https://panthema.net/2008/0901-stacktrace-demangled
  constexpr int max_frames = 16;
  void *stack_frames[max_frames];
  int num_frames = backtrace(stack_frames, max_frames); // Get stack addresses.

  // Get strings of trace.
  char **symbols = backtrace_symbols(stack_frames, num_frames);

  // Allocate string which will be filled with the demangled function name.
  // allocate string which will be filled with the demangled function name
  size_t funcnamesize = 256;
  char *funcname = static_cast<char *>(malloc(funcnamesize));

  // iterate over the returned symbol lines. skip the first, it is the
  // address of this function.
  for (int i = 1; i < num_frames; i++)
  {
    char *begin_name = 0, *begin_offset = 0, *end_offset = 0;

    // find parentheses and +address offset surrounding the mangled name:
    // ./module(function+0x15c) [0x8048a6d]
    for (char *p = symbols[i]; *p; ++p)
    {
      if (*p == '(')
      {
        begin_name = p;
      }
      else if (*p == '+')
      {
        begin_offset = p;
      }
      else if (*p == ')' && begin_offset)
      {
        end_offset = p;
        break;
      }
    }

    if (begin_name && begin_offset && end_offset && begin_name < begin_offset)
    {
      *begin_name++ = '\0';
      *begin_offset++ = '\0';
      *end_offset = '\0';

      // mangled name is now in [begin_name, begin_offset) and caller
      // offset in [begin_offset, end_offset). now apply
      // __cxa_demangle():

      int status;
      char *ret = abi::__cxa_demangle(begin_name, funcname, &funcnamesize,
                                      &status);
      if (status == 0)
      {
        funcname = ret; // use possibly realloc()-ed string
        fprintf(stderr,  COLOR_RED "  %s : %s+%s\n"  COLOR_RESET,
                symbols[i], funcname, begin_offset);
      }
      else
      {
        // demangling failed. Output function name as a C function with
        // no arguments.
        fprintf(stderr,  COLOR_RED "  %s : %s()+%s\n"  COLOR_RESET,
                symbols[i], begin_name, begin_offset);
      }
    }
    else
    {
      // couldn't parse the line? print the whole line.
      fprintf(stderr,  COLOR_RED "  %s\n"  COLOR_RESET, symbols[i]);
    }
  }

  free(funcname);
  free(symbols);
  exit(1);
}

}  // namespace utils

#define  ASSERT(condition) \
  do { \
    if (!(condition)) \
       utils::assert_fail(#condition, __PRETTY_FUNCTION__, __FILE__, __LINE__); \
  } while (false)
