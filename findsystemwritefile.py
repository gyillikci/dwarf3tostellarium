from ghidra.util.task import ConsoleTaskMonitor

prog = currentProgram
func_manager = prog.getFunctionManager()
listing = prog.getListing()

monitor = ConsoleTaskMonitor()

out_path = "C:/Users/TUTU/Desktop/workspace/dwarf3/firmware/ghidra_system_writefile.txt"
out = open(out_path, "w")

# Search for the SYSTEM module's own {cmd, handler} registration table by
# looking for any function containing several adjacent SYSTEM-range cmd
# values (13000-13010 = 0x32c8-0x32d2) as immediates -- same shape as the
# ASTRO module's registration function found earlier this session.
targets = set(range(0x32c8, 0x32d3))  # 13000-13010

ins_iter = listing.getInstructions(True)
hits_by_func = {}
count = 0
for ins in ins_iter:
    count += 1
    n = ins.getNumOperands()
    for i in range(n):
        try:
            objs = ins.getOpObjects(i)
        except:
            continue
        for o in objs:
            try:
                val = o.getValue() if hasattr(o, 'getValue') else None
            except:
                val = None
            if val in targets:
                f = func_manager.getFunctionContaining(ins.getAddress())
                fname = f.getName() if f else "???"
                hits_by_func.setdefault(fname, set()).add(val)

out.write("scanned %d instructions\n\n" % count)
for fname, vals in sorted(hits_by_func.items(), key=lambda kv: -len(kv[1])):
    out.write("%s : %d distinct SYSTEM-range values : %s\n" % (
        fname, len(vals), sorted(hex(v) for v in vals)))

out.close()
print("wrote " + out_path)
