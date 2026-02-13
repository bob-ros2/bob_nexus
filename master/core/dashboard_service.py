import os
import json
import yaml
import subprocess
import time
from datetime import datetime

class DashboardService:
    def __init__(self, root_dir):
        self.root_dir = os.path.abspath(root_dir)
        self.conf = self._load_conf()
        self.entities_dir = os.path.join(self.root_dir, "entities")

    def _load_conf(self):
        conf_path = os.path.join(self.root_dir, "master", "config", "conf.yaml")
        if os.path.exists(conf_path):
            with open(conf_path, 'r') as f:
                return yaml.safe_load(f)
        return {}

    def get_stats(self):
        master_name = self.conf.get("master_name", "Unknown Mastermind")
        
        running_masters = 0
        running_others = 0
        stopped = 0
        entities_list = []

        if os.path.exists(self.entities_dir):
            for category in os.listdir(self.entities_dir):
                cat_path = os.path.join(self.entities_dir, category)
                if not os.path.isdir(cat_path): continue
                
                for ent in os.listdir(cat_path):
                    ent_path = os.path.join(cat_path, ent)
                    manifest_path = os.path.join(ent_path, "manifest.json")
                    
                    status = "stopped"
                    identifier = "-"
                    if os.path.exists(manifest_path):
                        with open(manifest_path, 'r') as f:
                            m = json.load(f)
                            # Simple check for now, can be improved to use Deployer logic
                            if m.get("status") == "running":
                                status = "running"
                                if m.get("pid"):
                                    identifier = f"PID:{m['pid']}"
                                elif m.get("container_id"):
                                    identifier = f"CID:{m['container_id'][:12]}"
                    
                    entities_list.append({
                        "category": category,
                        "name": ent,
                        "status": status,
                        "id": identifier
                    })

                    if status == "running":
                        if category == "master":
                            running_masters += 1
                        else:
                            running_others += 1
                    else:
                        stopped += 1

        cpu = "N/A"
        mem = "N/A"
        try:
            # CPU Load (1 min avg)
            load = os.getloadavg()[0]
            cpu = f"{load:.1f}"
            # Memory %
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
                total = int(lines[0].split()[1])
                free = int(lines[1].split()[1])
                avail = int(lines[2].split()[1])
                used_pct = int((1 - (avail / total)) * 100)
                mem = f"{used_pct}%"
        except:
            pass

        return {
            "master_name": master_name,
            "running_masters": running_masters,
            "running_others": running_others,
            "stopped": stopped,
            "cpu": cpu,
            "mem": mem,
            "paths": {
                "root": self.root_dir,
                "config": os.path.join(self.root_dir, "master/config/conf.yaml"),
                "entities": self.entities_dir
            },
            "entities": entities_list,
            "timestamp": datetime.now().strftime("%H:%M:%S")
        }

if __name__ == "__main__":
    # Test output
    svc = DashboardService(".")
    print(json.dumps(svc.get_stats(), indent=2))
