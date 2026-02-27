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

# Try to import rich for markdown rendering (optional)
try:
    from rich.console import Console
    from rich.markdown import Markdown
    HAS_RICH = True
except ImportError:
    HAS_RICH = False

# --- ANSI Constants ---
CLR = "\033[K"          # Clear to end of line
MOVE = "\033[{line};{col}H"
BOLD = "\033[1m"
RESET = "\033[0m"
GREEN = "\033[32m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
RED = "\033[31m"

class AgentTalk:
    def __init__(self, entity_dir):
        self.entity_dir = Path(entity_dir).absolute()
        self.status_path = self.entity_dir / "status.json"
        self.inbox_path = self.entity_dir / "inbox.json"
        self.outbox_path = self.entity_dir / "outbox.json"
        self.log_path = self.entity_dir / "stdout.log"
        self.running = True
        self.last_status = {}
        self.log_pos = 0
        self.log_lines = []
        self.result_height = 5 # Default height for result area
        
        if HAS_RICH:
            self.console = Console()
        else:
            self.console = None
        
        # Verify if it looks like a Communication Registry
        if not self.inbox_path.exists() and not self.status_path.exists():
            print(f"{YELLOW}Warning: {self.entity_dir} doesn't look like a Nexus Registry.{RESET}")
            print(f"I will watch for incoming status/results here anyway.")

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
        # Position log area dynamically based on result height
        sys.stdout.write("\033[s")
        start_line = 13 + self.result_height 
        cols = os.get_terminal_size().columns
        rows = os.get_terminal_size().lines
        
        # Draw logs to fill the rest of the screen
        max_log_display = rows - start_line
        if max_log_display < 1: return # Screen too small
        
        display_lines = self.log_lines[-max_log_display:] if len(self.log_lines) > max_log_display else self.log_lines
        
        for i, line in enumerate(display_lines):
            sys.stdout.write(MOVE.format(line=start_line+i, col=1))
            sys.stdout.write(f"{RESET}{line[:cols-1]}{CLR}")
        sys.stdout.write("\033[u")
        sys.stdout.flush()

    def draw_outbox(self):
        try:
            if self.outbox_path.exists():
                with open(self.outbox_path, "r") as f:
                    data = json.load(f)
                    result = data.get("result", "")
                    if result:
                        # Draw to line 11 (above logs)
                        sys.stdout.write("\033[s")
                        sys.stdout.write(MOVE.format(line=11, col=1))
                        sys.stdout.write(f"{BOLD}{GREEN}--- FINAL RESULT (Tailing {self.result_height} lines) ---{RESET}{CLR}\n")
                        lines = result.splitlines()
                        
                        # Show the LAST lines of the result
                        display_lines = lines[-self.result_height:] if len(lines) > self.result_height else lines
                        
                        if HAS_RICH:
                            # Render via rich to string first, then handle line height
                            from io import StringIO
                            with StringIO() as buf:
                                console = Console(file=buf, force_terminal=True, width=os.get_terminal_size().columns-2)
                                console.print(Markdown(result))
                                rendered = buf.getvalue()
                                r_lines = rendered.splitlines()
                                r_display = r_lines[-self.result_height:] if len(r_lines) > self.result_height else r_lines
                                for i, l in enumerate(r_display):
                                    sys.stdout.write(f"{l}{CLR}\n")
                        else:
                            for i, l in enumerate(display_lines):
                                sys.stdout.write(f"{l[:os.get_terminal_size().columns-1]}{CLR}\n")
                        
                        sys.stdout.write("\033[u")
                        sys.stdout.flush()
            else:
                # Clear the area if no outbox exists
                sys.stdout.write("\033[s")
                sys.stdout.write(MOVE.format(line=11, col=1))
                sys.stdout.write(f"{YELLOW}Waiting for result...{RESET}{CLR}\n")
                for i in range(1, 6):
                    sys.stdout.write(MOVE.format(line=11+i, col=1) + CLR + "\n")
                sys.stdout.write("\033[u")
                sys.stdout.flush()
        except Exception as e:
            # Show parsing error briefly
            sys.stdout.write("\033[s")
            sys.stdout.write(MOVE.format(line=11, col=1))
            sys.stdout.write(f"{YELLOW}Result parsing... ({str(e)[:20]}){RESET}{CLR}")
            sys.stdout.write("\033[u")
            sys.stdout.flush()

    def update_loop(self):
        while self.running:
            self.draw_status()
            self.tail_logs()
            self.draw_outbox()
            time.sleep(1.0) # Faster updates

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
        # Initial screen wipe: Clear scrollback and screen (ANSI)
        sys.stdout.write("\033[H\033[2J\033[3J")
        sys.stdout.flush()

        if not HAS_RICH:
            print(f"{YELLOW}[Tip] Install 'rich' (pip install rich) for markdown rendering!{RESET}")
            time.sleep(1)
            sys.stdout.write("\033[H\033[2J\033[3J") # Clear again
            sys.stdout.flush()
        
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
                    # Clear old result first
                    if self.outbox_path.exists():
                        self.outbox_path.unlink()

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
    import argparse
    parser = argparse.ArgumentParser(description="Agent Talk TUI")
    parser.add_argument("dir", nargs="?", default=os.getcwd(), help="Entity directory")
    parser.add_argument("--height", type=int, default=5, help="Height of the result area")
    args = parser.parse_args()
    
    talk = AgentTalk(args.dir)
    talk.result_height = args.height
    talk.run()
