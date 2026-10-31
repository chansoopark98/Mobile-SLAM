// LD_PRELOAD sampling profiler for the UNMODIFIED production binary when perf(1) is not permitted (perf_event_paranoid=4).
// ITIMER_PROF (CPU-time) SIGPROF at SAMPLE_HZ (default 997); each sample stores up to 24 return addresses (glibc backtrace(), .eh_frame unwinding).
// On exit writes $SAMPLER_OUT: lines "base <hex load address of the main executable>" and "map <lo> <hi> <path>" (executable mappings), then one line per sample of hex addresses (leaf first).
// Resolve with resolve_samples.py. Dev-only, never linked into the product.
//   gcc -O1 -shared -fPIC -o sigprof_sampler.so sigprof_sampler.c -ldl -lrt
//   SAMPLE_HZ=<n> (default 997); SAMPLE_MONO=1 uses a CLOCK_MONOTONIC hrtimer instead of ITIMER_PROF (needed above ~1 kHz).
//   SAMPLER_OUT=<file> LD_PRELOAD=./sigprof_sampler.so <binary> ...
#define _GNU_SOURCE
#include <execinfo.h>
#include <link.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#define MAXS 2000000
#define DEPTH 24
static void* (*buf)[DEPTH];
static unsigned char* depth_;
static volatile long n;

static void handler(int sig) {
  (void)sig;
  long i = n;
  if (i >= MAXS) return;
  void* frames[DEPTH + 2];
  int d = backtrace(frames, DEPTH + 2);
  if (d <= 2) return;  // handler + sigreturn trampoline
  d -= 2;
  if (d > DEPTH) d = DEPTH;
  memcpy(buf[i], frames + 2, d * sizeof(void*));
  depth_[i] = (unsigned char)d;
  n = i + 1;
}

static int base_cb(struct dl_phdr_info* info, size_t sz, void* data) {
  (void)sz;
  if (info->dlpi_name[0] == 0) { *(unsigned long*)data = info->dlpi_addr; return 1; }
  return 0;
}

static void dump(void) {
  struct itimerval off = {{0, 0}, {0, 0}};
  setitimer(ITIMER_PROF, &off, NULL);
  const char* path = getenv("SAMPLER_OUT");
  if (!path) return;
  FILE* f = fopen(path, "w");
  if (!f) return;
  unsigned long base = 0;
  dl_iterate_phdr(base_cb, &base);
  fprintf(f, "base %lx\n", base);
  FILE* m = fopen("/proc/self/maps", "r");
  char ln[1024];
  while (m && fgets(ln, sizeof ln, m)) {
    unsigned long lo, hi; char perm[8], path[900] = "";
    if (sscanf(ln, "%lx-%lx %7s %*s %*s %*s %899s", &lo, &hi, perm, path) >= 3 && perm[2] == 'x' && path[0]) fprintf(f, "map %lx %lx %s\n", lo, hi, path);
  }
  if (m) fclose(m);
  for (long i = 0; i < n; i++) {
    for (int k = 0; k < depth_[i]; k++) fprintf(f, "%lx%c", (unsigned long)buf[i][k], k + 1 == depth_[i] ? '\n' : ' ');
  }
  fclose(f);
}

__attribute__((constructor)) static void init(void) {
  buf = calloc(MAXS, sizeof(*buf));
  depth_ = calloc(MAXS, 1);
  void* warm[4];
  backtrace(warm, 4);  // loads libgcc outside the handler
  struct sigaction sa;
  memset(&sa, 0, sizeof sa);
  sa.sa_handler = handler;
  sa.sa_flags = SA_RESTART;
  sigaction(SIGPROF, &sa, NULL);
  int hz = getenv("SAMPLE_HZ") ? atoi(getenv("SAMPLE_HZ")) : 997;
  if (getenv("SAMPLE_MONO")) {  // wall-clock hrtimer (rates above the kernel tick; use for single-threaded, CPU-bound, pinned runs only)
    struct sigevent sev;
    memset(&sev, 0, sizeof sev);
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGPROF;
    timer_t tid;
    timer_create(CLOCK_MONOTONIC, &sev, &tid);
    struct itimerspec its = {{0, 1000000000L / hz}, {0, 1000000000L / hz}};
    timer_settime(tid, 0, &its, NULL);
  } else {
    struct itimerval it = {{0, 1000000 / hz}, {0, 1000000 / hz}};
    setitimer(ITIMER_PROF, &it, NULL);
  }
  atexit(dump);
}
