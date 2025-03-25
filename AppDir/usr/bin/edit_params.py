#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox
import yaml
import os
import sys

# Determine the config file location.
if len(sys.argv) > 1:
    config_file = sys.argv[1]
else:
    # Fallback: look for a config folder next to the script.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, "config", "params.yaml")

if not os.path.exists(config_file):
    messagebox.showerror("Error", f"Config file not found:\n{config_file}")
    exit(1)


# Now load the config file.
try:
    with open(config_file, 'r') as f:
        config_data = yaml.safe_load(f)
except Exception as e:
    messagebox.showerror("Error", f"Error reading YAML:\n{e}")
    exit(1)

# Prepare a list of (key_path, value) tuples.
fields = []

def traverse_dict(d, path=""):
    for k, v in d.items():
        new_path = f"{path}.{k}" if path else k
        if isinstance(v, dict):
            traverse_dict(v, new_path)
        else:
            fields.append((new_path, v))

traverse_dict(config_data)

# Create the main window for editing.
editor = tk.Tk()
editor.title("Edit Parameters")

entries = {}
row = 0

for key, value in fields:
    lbl = tk.Label(editor, text=key)
    lbl.grid(row=row, column=0, padx=5, pady=5, sticky="e")
    entry = tk.Entry(editor, width=40)
    entry.insert(0, str(value))
    entry.grid(row=row, column=1, padx=5, pady=5)
    entries[key] = entry
    row += 1

def on_ok():
    # Update the original config_data using the entries.
    for key, entry in entries.items():
        keys = key.split(".")
        current = config_data
        for subkey in keys[:-1]:
            current = current[subkey]
        # Update only the leaf value.
        current[keys[-1]] = entry.get()
    try:
        with open(config_file, 'w') as f:
            yaml.dump(config_data, f)
        messagebox.showinfo("Success", "Parameters updated successfully.")
        editor.destroy()
    except Exception as e:
        messagebox.showerror("Error", f"Error writing YAML:\n{e}")

def on_cancel():
    if messagebox.askokcancel("Cancel", "Cancel without saving changes?"):
        editor.destroy()

# Buttons for OK and Cancel.
button_frame = tk.Frame(editor)
button_frame.grid(row=row, column=0, columnspan=2, pady=10)
ok_btn = tk.Button(button_frame, text="OK", command=on_ok)
ok_btn.pack(side="left", padx=5)
cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel)
cancel_btn.pack(side="left", padx=5)

editor.mainloop()
