#!/usr/bin/env python3
import sys
import os
import yaml
from PyQt5 import QtWidgets, QtCore

# Function to traverse the dictionary and get a list of (key_path, value) tuples.
def traverse_dict(d, path=""):
    fields = []
    for key, value in d.items():
        new_path = f"{path}.{key}" if path else key
        if isinstance(value, dict):
            fields.extend(traverse_dict(value, new_path))
        else:
            fields.append((new_path, value))
    return fields

class EditParametersDialog(QtWidgets.QDialog):
    def __init__(self, config_file, config_data, parent=None):
        super(EditParametersDialog, self).__init__(parent)
        self.setWindowTitle("Edit Parameters")
        self.config_file = config_file
        self.config_data = config_data

        # Create a grid layout for the fields.
        self.grid_layout = QtWidgets.QGridLayout()
        self.edit_fields = {}  # key: QLineEdit

        fields = traverse_dict(config_data)
        row = 0
        for key, value in fields:
            label = QtWidgets.QLabel(key)
            line_edit = QtWidgets.QLineEdit(str(value))
            self.grid_layout.addWidget(label, row, 0)
            self.grid_layout.addWidget(line_edit, row, 1)
            self.edit_fields[key] = line_edit
            row += 1

        # OK and Cancel buttons.
        button_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.on_ok)
        button_box.rejected.connect(self.reject)

        # Set the main layout.
        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(self.grid_layout)
        main_layout.addWidget(button_box)
        self.setLayout(main_layout)

    def on_ok(self):
        # Update the config_data with the new values.
        for key, line_edit in self.edit_fields.items():
            new_value = line_edit.text()
            keys = key.split(".")
            current = self.config_data
            for subkey in keys[:-1]:
                current = current[subkey]
            current[keys[-1]] = new_value
        try:
            with open(self.config_file, 'w') as f:
                yaml.dump(self.config_data, f)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Error writing YAML:\n{e}")
            return
        self.accept()

def main():
    # Determine the config file location.
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_file = os.path.join(script_dir, "config", "params.yaml")
    
    if not os.path.exists(config_file):
        QtWidgets.QMessageBox.critical(None, "Error", f"Config file not found:\n{config_file}")
        sys.exit(1)
    
    try:
        with open(config_file, 'r') as f:
            config_data = yaml.safe_load(f)
    except Exception as e:
        QtWidgets.QMessageBox.critical(None, "Error", f"Error reading YAML:\n{e}")
        sys.exit(1)

    app = QtWidgets.QApplication(sys.argv)
    dialog = EditParametersDialog(config_file, config_data)
    if dialog.exec_() == QtWidgets.QDialog.Accepted:
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
