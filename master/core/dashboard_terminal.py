import os
import sys
import time

# Ensure we can import DashboardService
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "core"))
from dashboard_service import DashboardService

# ANSI Colors
G = '\033[0;32m'
C = '\033[0;36m'
R = '\033[0;31m'
Y = '\033[1;33m'
P = '\033[0;35m'
W = '\033[1;37m'
B = '\033[1m'
NC = '\033[0m'

def print_dashboard(root_dir):
    svc = DashboardService(root_dir)
    data = svc.get_stats()

    os.system('clear')

    # Logo Area
    print(f"{C}{B}")
    print("  ██████╗  ██████╗ ██████╗     ███╗   ██╗███████╗██╗  ██╗██╗   ██╗███████╗")
    print("  ██╔══██╗██╔═══██╗██╔══██╗    ████╗  ██║██╔════╝╚██╗██╔╝██║   ██║██╔════╝")
    print("  ██████╔╝██║   ██║██████╔╝    ██╔██╗ ██║█████╗   ╚███╔╝ ██║   ██║███████╗")
    print("  ██╔══██╗██║   ██║██╔══██╗    ██║╚██╗██║██╔══╝   ██╔██╗ ██║   ██║╚════██║")
    print("  ██████╔╝╚██████╔╝██████╔╝    ██║ ╚████║███████╗██╔╝ ██╗╚██████╔╝███████║")
    print("  ╚═════╝  ╚═════╝ ╚═════╝     ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝")
    print(f"                            {G}EXPERIMENT 7! - THE AWAKENING{NC}")
    print(f"{NC}")

    # Stats Area
    p = data["paths"]
    print(f"{P}--- System Awareness --------------------------------------------------{NC}")
    print(f" {B}MASTER:{NC} {Y}{data['master_name']}{NC} | {B}MODE:{NC} {data['cpu']} CPU / {data['mem']} MEM | {B}TIME:{NC} {data['timestamp']}")
    print(f" {B}PATH:{NC}   {p['root']}")
    print(f" {B}CONFIG:{NC} {p['config']}")

    # Counters Area
    print(f" {B}SWARM:{NC}  {G}{data['running_masters']} Masters Running{NC} | {C}{data['running_others']} Others Running{NC} | {R}{data['stopped']} Stopped{NC}")
    print(f"{P}----------------------------------------------------------------------{NC}")

    # Entities Table
    print(f"{B}{'CATEGORY':<15} {'NAME':<15} {'STATUS':<10} {'IDENTIFIER':<15}{NC}")
    print("-" * 65)

    # Sort entities: Master first, then others
    sorted_ents = sorted(data["entities"], key=lambda x: (x["category"] != "master", x["name"]))

    for ent in sorted_ents:
        status_color = G if ent["status"] == "running" else R
        print(f"{ent['category']:<15} {ent['name']:<15} {status_color}{ent['status']:<10}{NC} {ent['id']:<15}")

    print(f"\n{Y}Press Ctrl+C to disconnect from Nexus.{NC}")

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "loop"
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    if mode == "status":
        print_dashboard(root)
    else:
        try:
            while True:
                print_dashboard(root)
                time.sleep(2)
        except KeyboardInterrupt:
            print(f"\n{Y}[*] Nexus link terminated.{NC}")
