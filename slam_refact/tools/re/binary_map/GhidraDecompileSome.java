// Ghidra headless post-script: decompile selected wasm functions with a long timeout.
// Functions are selected by wasm index (unnamed_function_<N> or the export name of N is resolved
// through the entry address passed alongside: "<index>:<hex entry>").
// Usage: analyzeHeadless <proj_dir> <proj> -import xr-slam.wasm -deleteProject \
//          -scriptPath slam_refact/tools/re/binary_map -postScript GhidraDecompileSome.java \
//          <out_dir> <timeout_seconds> <index:entryhex> [<index:entryhex> ...]
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileOptions;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;

import java.io.File;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;

public class GhidraDecompileSome extends GhidraScript {
  @Override
  public void run() throws Exception {
    String[] args = getScriptArgs();
    File out = new File(args[0]);
    File cdir = new File(out, "c");
    cdir.mkdirs();
    int timeout = Integer.parseInt(args[1]);
    DecompInterface dec = new DecompInterface();
    DecompileOptions opts = new DecompileOptions();
    opts.setMaxPayloadMBytes(512);
    dec.setOptions(opts);
    dec.openProgram(currentProgram);
    try (PrintWriter idx = new PrintWriter(new File(out, "functions.jsonl"), StandardCharsets.UTF_8)) {
      for (int i = 2; i < args.length; i++) {
        String[] parts = args[i].split(":");
        Address a = currentProgram.getAddressFactory().getDefaultAddressSpace().getAddress(parts[1]);
        Function f = getFunctionAt(a);
        if (f == null) {
          idx.println(String.format("{\"index\":%s,\"entry\":\"%s\",\"error\":\"no function\"}", parts[0], parts[1]));
          continue;
        }
        long t0 = System.currentTimeMillis();
        DecompileResults res = dec.decompileFunction(f, timeout, monitor);
        long dt = System.currentTimeMillis() - t0;
        boolean done = res.decompileCompleted();
        String file = String.format("%s_%s.c", f.getEntryPoint().toString(), f.getName().replaceAll("[^A-Za-z0-9_.-]", "_"));
        if (done) {
          try (PrintWriter w = new PrintWriter(new File(cdir, file), StandardCharsets.UTF_8)) {
            w.print(res.getDecompiledFunction().getC());
          }
        }
        String err = done ? "" : String.valueOf(res.getErrorMessage()).replace("\"", "'").replace("\n", " ");
        idx.println(String.format(
            "{\"index\":%s,\"name\":\"%s\",\"entry\":\"%s\",\"decompiled\":%b,\"file\":\"%s\",\"millis\":%d,\"error\":\"%s\"}",
            parts[0], f.getName(), f.getEntryPoint(), done, done ? file : "", dt, err));
        idx.flush();
        println(String.format("GhidraDecompileSome: %s done=%b %d ms", f.getName(), done, dt));
      }
    }
  }
}
