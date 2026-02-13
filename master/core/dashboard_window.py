import os
import sys
import tkinter as tk
from tkinter import ttk

# Ensure we can import DashboardService
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "core"))
from dashboard_service import DashboardService


class NexusDashboardWindow:
    def __init__(self, root_dir):
        self.root_dir = root_dir
        self.svc = DashboardService(self.root_dir)

        self.root = tk.Tk()
        self.root.title("BOB NEXUS - EXP7! Dashboard")
        self.root.geometry("1100x700")
        self.root.configure(bg="#000000")

        # Style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "Treeview",
            background="#000000",
            foreground="#ffffff",
            fieldbackground="#000000",
            rowheight=35,
            font=("Consolas", 11),
        )
        style.configure(
            "Treeview.Heading",
            background="#1a1a1a",
            foreground="#00ffff",
            font=("Helvetica", 11, "bold"),
            relief="flat",
        )
        style.map("Treeview", background=[("selected", "#00ffff")], foreground=[("selected", "black")])

        self._setup_ui()
        self.update_data()
        self.root.mainloop()

    def _setup_ui(self):
        # Header area
        self.header_frame = tk.Frame(self.root, bg="#000000", pady=15)
        self.header_frame.pack(fill=tk.X)

        self.title_label = tk.Label(
            self.header_frame,
            text="BOB NEXUS",
            font=("Helvetica", 24, "bold"),
            fg="#00ffff",
            bg="#000000",
        )
        self.title_label.pack()
        self.subtitle_label = tk.Label(
            self.header_frame,
            text="EXPERIMENT 7! - THE AWAKENING",
            font=("Helvetica", 12, "bold"),
            fg="#ffffff",
            bg="#000000",
        )
        self.subtitle_label.pack()

        # Stats bar
        self.stats_frame = tk.Frame(self.root, bg="#00ffff", pady=8)
        self.stats_frame.pack(fill=tk.X)

        self.stats_label = tk.Label(
            self.stats_frame,
            text="Loading Stats...",
            font=("Consolas", 12, "bold"),
            fg="#000000",
            bg="#00ffff",
        )
        self.stats_label.pack()

        # Paths Area
        self.paths_frame = tk.Frame(self.root, bg="#000000", pady=20, padx=30)
        self.paths_frame.pack(fill=tk.X)

        # Master Label (Not copyable)
        tk.Label(
            self.paths_frame, text="MASTER:", font=("Helvetica", 10, "bold"), fg="#888888", bg="#000000"
        ).grid(row=0, column=0, sticky="w")
        self.master_label = tk.Label(
            self.paths_frame, text="", font=("Consolas", 11), fg="#ffffff", bg="#000000"
        )
        self.master_label.grid(row=0, column=1, padx=10, sticky="w")

        def create_copy_row(parent, label_text, row):
            tk.Label(
                parent, text=f"{label_text}:", font=("Helvetica", 10, "bold"), fg="#888888", bg="#000000"
            ).grid(row=row, column=0, sticky="w", pady=5)
            var = tk.StringVar()
            ent = tk.Entry(
                parent,
                textvariable=var,
                font=("Consolas", 10),
                fg="#000000",
                bg="#ffffff",
                insertbackground="black",
                relief=tk.FLAT,
                width=90,
            )
            ent.grid(row=row, column=1, padx=10, pady=5, sticky="w")
            ent.configure(state="readonly")

            def copy():
                self.root.clipboard_clear()
                self.root.clipboard_append(var.get())

            btn = tk.Button(
                parent,
                text="COPY",
                command=copy,
                font=("Helvetica", 8, "bold"),
                bg="#333333",
                fg="white",
                relief="flat",
                padx=10,
            )
            btn.grid(row=row, column=2, padx=10)
            return var

        self.root_var = create_copy_row(self.paths_frame, "ROOT", 1)
        self.config_var = create_copy_row(self.paths_frame, "CONFIG", 2)

        # Table area
        self.table_frame = tk.Frame(self.root, bg="#000000", padx=30, pady=10)
        self.table_frame.pack(fill=tk.BOTH, expand=True)

        columns = ("category", "name", "status", "id")
        self.tree = ttk.Treeview(self.table_frame, columns=columns, show="headings")
        self.tree.heading("category", text="CATEGORY")
        self.tree.heading("name", text="NAME")
        self.tree.heading("status", text="STATUS")
        self.tree.heading("id", text="IDENTIFIER")

        self.tree.column("category", width=150)
        self.tree.column("name", width=200)
        self.tree.column("status", width=120)
        self.tree.column("id", width=200)

        self.tree.pack(fill=tk.BOTH, expand=True)

        # Bottom area
        self.bottom_frame = tk.Frame(self.root, bg="#000000", pady=20)
        self.bottom_frame.pack(fill=tk.X)

        self.refresh_btn = tk.Button(
            self.bottom_frame,
            text="FORCE REFRESH",
            command=self.update_data,
            bg="#00ffff",
            fg="black",
            font=("Helvetica", 10, "bold"),
            relief="flat",
            padx=20,
        )
        self.refresh_btn.pack(side=tk.RIGHT, padx=30)

        self.auto_refresh_var = tk.BooleanVar(value=True)
        self.auto_refresh_check = tk.Checkbutton(
            self.bottom_frame,
            text="AUTO-REFRESH (5S)",
            variable=self.auto_refresh_var,
            font=("Helvetica", 9, "bold"),
            bg="#000000",
            fg="#ffffff",
            selectcolor="#000000",
            activebackground="#000000",
            activeforeground="#ffffff",
        )
        self.auto_refresh_check.pack(side=tk.LEFT, padx=30)

    def update_data(self):
        data = self.svc.get_stats()

        # Update Stats
        stats_text = f"CPU: {data['cpu']} | MEM: {data['mem']} | MASTERS: {data['running_masters']} | OTHERS: {data['running_others']} | STOPPED: {data['stopped']}"
        self.stats_label.config(text=stats_text)

        # Update Paths
        self.master_label.config(text=data["master_name"])
        self.root_var.set(data["paths"]["root"])
        self.config_var.set(data["paths"]["config"])

        # Update Table
        for item in self.tree.get_children():
            self.tree.delete(item)

        sorted_ents = sorted(data["entities"], key=lambda x: (x["category"] != "master", x["name"]))
        for ent in sorted_ents:
            self.tree.insert("", tk.END, values=(ent["category"], ent["name"], ent["status"], ent["id"]))

        # Schedule next update
        if self.auto_refresh_var.get():
            self.root.after(5000, self.update_data)

if __name__ == "__main__":
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    NexusDashboardWindow(root)
