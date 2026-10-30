// Ghidra headless post-script: export decompiled C, callees and string refs for every function.
// Usage: analyzeHeadless <proj_dir> <proj> -import xr-slam.wasm -scriptPath slam_refact/tools/re \
//          -postScript GhidraExportAll.java <out_dir>
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.Data;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.symbol.Reference;

import java.io.File;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.util.Set;
import java.util.TreeSet;

public class GhidraExportAll extends GhidraScript {
  private static String esc(String s) {
    StringBuilder b = new StringBuilder();
    for (char c : s.toCharArray()) {
      switch (c) {
        case '"': b.append("\\\""); break;
        case '\\': b.append("\\\\"); break;
        case '\n': b.append("\\n"); break;
        case '\r': b.append("\\r"); break;
        case '\t': b.append("\\t"); break;
        default:
          if (c < 0x20) b.append(String.format("\\u%04x", (int) c)); else b.append(c);
      }
    }
    return b.toString();
  }

  @Override
  public void run() throws Exception {
    String[] args = getScriptArgs();
    File out = new File(args.length > 0 ? args[0] : "ghidra-export");
    File cdir = new File(out, "c");
    cdir.mkdirs();
    DecompInterface dec = new DecompInterface();
    dec.openProgram(currentProgram);
    int ok = 0, fail = 0;
    try (PrintWriter idx = new PrintWriter(new File(out, "functions.jsonl"), StandardCharsets.UTF_8)) {
      for (Function f : currentProgram.getFunctionManager().getFunctions(true)) {
        if (monitor.isCancelled()) break;
        Set<String> callees = new TreeSet<>();
        for (Function c : f.getCalledFunctions(monitor)) callees.add(c.getName());
        Set<String> strings = new TreeSet<>();
        for (Instruction ins : currentProgram.getListing().getInstructions(f.getBody(), true)) {
          for (Reference r : ins.getReferencesFrom()) {
            Data d = getDataAt(r.getToAddress());
            if (d != null && d.hasStringValue()) strings.add(String.valueOf(d.getValue()));
          }
        }
        DecompileResults res = dec.decompileFunction(f, 120, monitor);
        boolean done = res.decompileCompleted();
        String safe = f.getName().replaceAll("[^A-Za-z0-9_.-]", "_");
        String file = String.format("%s_%s.c", f.getEntryPoint().toString(), safe);
        if (done) {
          try (PrintWriter w = new PrintWriter(new File(cdir, file), StandardCharsets.UTF_8)) {
            w.print(res.getDecompiledFunction().getC());
          }
          ok++;
        } else {
          fail++;
        }
        StringBuilder j = new StringBuilder();
        j.append("{\"name\":\"").append(esc(f.getName())).append("\",\"entry\":\"")
            .append(f.getEntryPoint()).append("\",\"size\":").append(f.getBody().getNumAddresses())
            .append(",\"decompiled\":").append(done).append(",\"file\":\"").append(done ? file : "")
            .append("\",\"callees\":[");
        boolean first = true;
        for (String c : callees) { j.append(first ? "" : ",").append('"').append(esc(c)).append('"'); first = false; }
        j.append("],\"strings\":[");
        first = true;
        for (String s : strings) { j.append(first ? "" : ",").append('"').append(esc(s)).append('"'); first = false; }
        j.append("]}");
        idx.println(j);
      }
    }
    println(String.format("GhidraExportAll: decompiled=%d failed=%d", ok, fail));
  }
}
