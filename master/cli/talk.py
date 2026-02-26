#!/usr/bin/env python3
import os
import sys
import json
import time
import datetime
import threading
from pathlib import Path

# Try to import readline for better input handling (history, cursor movement)
try:
    import readline
except ImportError:
    pass

# --- ANSI Constants ---
CLR = "\033[K"          # Clear to end of line
MOVE = "\033[{line};{col}H"
BOLD = "\033[1m"
RESET = "\033[0m"
GREEN = "\033[32m"
CYAN = "\033[36m"
YELLOW = "\033[33m"

class AgentTalk:
    def __init__(self, entity_dir):
        self.entity_dir = Path(entity_dir).absolute()
        self.status_path = self.entity_dir / "status.json"
        self.inbox_path = self.entity_dir / "inbox.json"
        self.log_path = self.entity_dir / "stdout.log"
        self.running = True
        self.last_status = {}
        self.log_pos = 0
        self.log_lines = []
        
        # Verify if it looks like an entity directory
        if not (self.entity_dir / "agent.yaml").exists() and not self.status_path.exists():
            print(f"{YELLOW}Warning: {self.entity_dir} does not contain agent.yaml or status.json.{RESET}")
            print(f"I will still try to work here, but I might not find what I need.")

    def format_timestamp(self, ts):
        if not ts: return "--:--:--.---"
        # Handle both float and string timestamps if necessary
        try:
            dt = datetime.datetime.fromtimestamp(float(ts))
            return dt.strftime("%H:%M:%S.%f")[:-3]
        except:
            return str(ts)

    def draw_status(self):
        try:
            if self.status_path.exists():
                with open(self.status_path, "r") as f:
                    data = json.load(f)
                    self.last_status = data
            
            ts = self.last_status.get("last_heartbeat", 0)
            state = self.last_status.get("state", "unknown")
            model = self.last_status.get("api_model", "n/a")
            thought = self.last_status.get("thought", "")

            # Draw to top (save cursor first)
            sys.stdout.write("\033[s") 
            sys.stdout.write(MOVE.format(line=1, col=1))
            sys.stdout.write(f"{BOLD}{CYAN}--- AGENT: {self.entity_dir.name} ---{RESET} {CLR}\n")
            sys.stdout.write(f"[{BOLD}{state.upper()}{RESET}] | Model: {YELLOW}{model}{RESET} | Heartbeat: {GREEN}{self.format_timestamp(ts)}{RESET} {CLR}\n")
            sys.stdout.write(f"Thought: {thought[:100]}{'...' if len(thought)>100 else ''} {CLR}\n")
            sys.stdout.write(f"{CYAN}{'='*60}{RESET} {CLR}\n")
            sys.stdout.write("\033[u") # Restore cursor
            sys.stdout.flush()
        except:
            pass

    def tail_logs(self):
        try:
            if not self.log_path.exists(): return
            with open(self.log_path, "r") as f:
                f.seek(0, os.SEEK_END)
                if self.log_pos == 0: 
                    self.log_pos = max(0, f.tell() - 4000) # Load a bit of history
                
                f.seek(self.log_pos)
                new_data = f.read()
                self.log_pos = f.tell()
                
                if new_data:
                    lines = new_data.splitlines()
                    for line in lines:
                        if line.strip():
                            self.log_lines.append(line)
                    
                    if len(self.log_lines) > 20:
                        self.log_lines = self.log_lines[-20:]
                    self.draw_logs()
        except: pass

    def draw_logs(self):
        # Position log area (starting at line 12)
        sys.stdout.write("\033[s")
        start_line = 12
        cols = os.get_terminal_size().columns
        for i, line in enumerate(self.log_lines):
            sys.stdout.write(MOVE.format(line=start_line+i, col=1))
            sys.stdout.write(f"{RESET}{line[:cols-1]}{CLR}")
        sys.stdout.write("\033[u")
        sys.stdout.flush()

    def update_loop(self):
        while self.running:
            self.draw_status()
            self.tail_logs()
            time.sleep(1.5)

    def get_multi_line_input(self):
        lines = []
        sys.stdout.write(MOVE.format(line=6, col=1))
        sys.stdout.write(f"{BOLD}{YELLOW}Enter Mission (Empty line to send, 'exit' to quit):{RESET}{CLR}\n")
        
        while True:
            try:
                sys.stdout.write(f"{CYAN}> {RESET}{CLR}")
                sys.stdout.flush()
                line = input()
                if not line:
                    if lines: break
                    else: continue
                if line.lower() in ["exit", "quit"] and not lines:
                    return "exit"
                lines.append(line)
            except EOFError:
                return "exit"
            except KeyboardInterrupt:
                return "exit"
        
        return "\n".join(lines).strip()

    def run(self):
        # Initial space
        print("\n" * 30) 
        
        # Start background update thread
        updater = threading.Thread(target=self.update_loop, daemon=True)
        updater.start()

        while self.running:
            mission = self.get_multi_line_input()
            
            if mission == "exit":
                self.running = False
                break

            if mission:
                try:
                    with open(self.inbox_path, "w") as f:
                        json.dump({"mission": mission}, f)
                    
                    sys.stdout.write(MOVE.format(line=8, col=1))
                    sys.stdout.write(f"{GREEN}Mission sent to inbox.json!{RESET}{CLR}")
                    time.sleep(0.8)
                    sys.stdout.write(MOVE.format(line=8, col=1))
                    sys.stdout.write(CLR)
                    
                    # Clear input area
                    for i in range(6, 11):
                        sys.stdout.write(MOVE.format(line=i, col=1) + CLR)
                except Exception as e:
                    sys.stdout.write(MOVE.format(line=8, col=1))
                    sys.stdout.write(f"{RED}Error sending mission: {e}{RESET}{CLR}")
                    time.sleep(2)

        print(f"\n{BOLD}Exiting Talk TUI.{RESET}")

if __name__ == "__main__":
    target_dir = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    AgentTalk(target_dir).run()
