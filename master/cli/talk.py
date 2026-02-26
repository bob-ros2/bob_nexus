#!/usr/bin/env python3
import os
import sys
import json
import time
import datetime
import threading
import select
from pathlib import Path

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
        self.entity_dir = Path(entity_dir)
        self.status_path = self.entity_dir / "status.json"
        self.inbox_path = self.entity_dir / "inbox.json"
        self.log_path = self.entity_dir / "stdout.log"
        self.running = True
        self.last_status = {}
        self.log_pos = 0
        self.log_lines = []

    def format_timestamp(self, ts):
        if not ts: return "--:--:--.---"
        return datetime.datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]

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
            sys.stdout.write(f"{BOLD}{CYAN}--- STATUS ---{RESET} {CLR}\n")
            sys.stdout.write(f"[{BOLD}{state.upper()}{RESET}] | Model: {YELLOW}{model}{RESET} | Heartbeat: {GREEN}{self.format_timestamp(ts)}{RESET} {CLR}\n")
            sys.stdout.write(f"Thought: {thought[:100]}{'...' if len(thought)>100 else ''} {CLR}\n")
            sys.stdout.write(f"{CYAN}{'='*60}{RESET} {CLR}\n")
            sys.stdout.write("\033[u") # Restore cursor
            sys.stdout.flush()
        except Exception as e:
            pass

    def tail_logs(self):
        try:
            if not self.log_path.exists(): return
            with open(self.log_path, "r") as f:
                f.seek(0, os.SEEK_END)
                if self.log_pos == 0: self.log_pos = max(0, f.tell() - 2000)
                f.seek(self.log_pos)
                new_data = f.read()
                self.log_pos = f.tell()
                if new_data:
                    lines = new_data.splitlines()
                    self.log_lines.extend(lines)
                    if len(self.log_lines) > 15:
                        self.log_lines = self.log_lines[-15:]
                    self.draw_logs()
        except: pass

    def draw_logs(self):
        # Position log area (let's say line 12 to 27)
        sys.stdout.write("\033[s")
        start_line = 12
        for i, line in enumerate(self.log_lines):
            sys.stdout.write(MOVE.format(line=start_line+i, col=1))
            sys.stdout.write(f"{RESET}{line[:os.get_terminal_size().columns]}{CLR}")
        sys.stdout.write("\033[u")
        sys.stdout.flush()

    def update_loop(self):
        while self.running:
            self.draw_status()
            self.tail_logs()
            time.sleep(2)

    def run(self):
        print("\n\n\n\n\n") # Buffer for header
        print(f"{BOLD}Agent Talk TUI started for {self.entity_dir.name}{RESET}")
        print(f"Type mission and press Enter. (Empty line to send multi-line)\n")
        
        # Start background update thread
        updater = threading.Thread(target=self.update_loop, daemon=True)
        updater.start()

        while self.running:
            try:
                sys.stdout.write(MOVE.format(line=6, col=1))
                sys.stdout.write(f"{BOLD}{YELLOW}Your next Aufgabe (Mission):{RESET}{CLR}\n")
                sys.stdout.write("> ")
                sys.stdout.flush()

                # Multi-line input logic
                lines = []
                while True:
                    line = sys.stdin.readline()
                    if not line: break
                    clean = line.strip()
                    if not clean and lines: break # Second empty line sends
                    lines.append(clean)
                    if not any(lines): continue # Skip initial empty lines
                    if len(lines) == 1 and clean: break # Fast send for single line

                mission = " ".join(lines).strip()
                if mission.lower() in ["quit", "exit"]:
                    self.running = False
                    break

                if mission:
                    with open(self.inbox_path, "w") as f:
                        json.dump({"mission": mission}, f)
                    sys.stdout.write(MOVE.format(line=8, col=1))
                    sys.stdout.write(f"{GREEN}Mission sent!{RESET}{CLR}")
                    time.sleep(1)
                    sys.stdout.write(MOVE.format(line=8, col=1))
                    sys.stdout.write(CLR)

            except KeyboardInterrupt:
                self.running = False
                break

        print("\nExiting Talk TUI.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: talk.py <entity_dir>")
        sys.exit(1)
    
    AgentTalk(sys.argv[1]).run()
