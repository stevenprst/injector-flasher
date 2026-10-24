"""A small window to load an app image into the injector flasher over USB and to watch what it holds.

    python injector_gui.py

Pick the COM port of the injector, press Connect, choose the .bin of the app and the node id of the board it is for,
press Send. The injector keeps the image in its flash; its LED shows that it holds one, and its two buttons inject it
into the CAN bus and delete it.
"""

import queue
import threading
import tkinter as tk
from tkinter import filedialog, ttk

import serial
import serial.tools.list_ports

import injector_link

STATUS_POLL_MS = 500
RESULT_POLL_MS = 100


class InjectorWindow:
    def __init__(self, root):
        self.root = root
        self.link = None
        self.busy = False		# a send is running, so no status polling
        self.tasks = queue.Queue()
        self.results = queue.Queue()

        root.title("Injector flasher")
        root.columnconfigure(1, weight=1)

        self.port_name = tk.StringVar()
        self.file_name = tk.StringVar()
        self.node_id = tk.IntVar(value=1)
        self.status_text = tk.StringVar(value="not connected")
        self.message_text = tk.StringVar()

        ttk.Label(root, text="Port").grid(row=0, column=0, sticky="w", padx=8, pady=4)
        self.port_box = ttk.Combobox(root, textvariable=self.port_name, state="readonly")
        self.port_box.grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(root, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        self.connect_button = ttk.Button(root, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=8)

        ttk.Label(root, text="App image (.bin)").grid(row=1, column=0, sticky="w", padx=8, pady=4)
        ttk.Entry(root, textvariable=self.file_name).grid(row=1, column=1, sticky="ew", padx=8)
        ttk.Button(root, text="Browse", command=self.browse).grid(row=1, column=2, padx=4)

        ttk.Label(root, text="Node id").grid(row=2, column=0, sticky="w", padx=8, pady=4)
        ttk.Spinbox(root, from_=0, to=255, textvariable=self.node_id, width=6).grid(row=2, column=1, sticky="w", padx=8)
        self.send_button = ttk.Button(root, text="Send to injector", command=self.send, state="disabled")
        self.send_button.grid(row=2, column=3, padx=8)

        self.progress = ttk.Progressbar(root, maximum=1)
        self.progress.grid(row=3, column=0, columnspan=4, sticky="ew", padx=8, pady=4)

        ttk.Label(root, textvariable=self.status_text, justify="left").grid(
            row=4, column=0, columnspan=4, sticky="w", padx=8, pady=4)
        ttk.Label(root, textvariable=self.message_text, foreground="#a00000").grid(
            row=5, column=0, columnspan=4, sticky="w", padx=8, pady=4)

        threading.Thread(target=self.worker, daemon=True).start()
        self.refresh_ports()
        self.root.after(RESULT_POLL_MS, self.poll_results)
        self.root.after(STATUS_POLL_MS, self.poll_status)

    # ---- the worker thread owns the serial port, so the window never waits for the injector ----

    def worker(self):
        while True:
            task = self.tasks.get()

            try:
                self.results.put(("done", task()))
            except (injector_link.LinkError, serial.SerialException, OSError) as error:
                self.results.put(("error", str(error)))

    def poll_results(self):
        while not self.results.empty():
            kind, value = self.results.get()

            if kind == "error":
                self.message_text.set(value)
                self.busy = False
                self.send_button.config(state="normal" if self.link else "disabled")
            elif kind == "progress":
                sent, total = value
                self.progress.config(maximum=total, value=sent)
            elif isinstance(value, injector_link.Status):
                self.status_text.set("\n".join(injector_link.describe_status(value)))
            elif value == "sent":
                self.message_text.set("")
                self.busy = False
                self.send_button.config(state="normal")

        self.root.after(RESULT_POLL_MS, self.poll_results)

    def poll_status(self):
        if self.link and not self.busy:
            self.tasks.put(self.link.get_status)

        self.root.after(STATUS_POLL_MS, self.poll_status)

    # ---- buttons ----

    def refresh_ports(self):
        names = [port.device for port in serial.tools.list_ports.comports()]
        self.port_box["values"] = names

        if names and not self.port_name.get():
            self.port_name.set(names[0])

    def toggle_connection(self):
        if self.link:
            self.link = None
            self.connect_button.config(text="Connect")
            self.send_button.config(state="disabled")
            self.status_text.set("not connected")
            return

        try:
            port = serial.Serial(self.port_name.get(), 115200, timeout=0.1)
        except (serial.SerialException, OSError) as error:
            self.message_text.set(str(error))
            return

        self.link = injector_link.InjectorLink(port)
        self.connect_button.config(text="Disconnect")
        self.send_button.config(state="normal")
        self.message_text.set("")

    def browse(self):
        name = filedialog.askopenfilename(filetypes=[("App image", "*.bin"), ("All files", "*.*")])

        if name:
            self.file_name.set(name)

    def send(self):
        try:
            with open(self.file_name.get(), "rb") as image_file:
                data = image_file.read()

            injector_link.check_image(data)
            node_id = self.node_id.get()
        except (OSError, injector_link.LinkError, tk.TclError) as error:
            self.message_text.set(str(error))
            return

        self.busy = True
        self.send_button.config(state="disabled")
        self.message_text.set("")
        self.progress.config(value=0)

        # tkinter is not thread safe, so the worker only queues the progress and the window draws it
        def show_progress(sent, total):
            self.results.put(("progress", (sent, total)))

        def send_task():
            self.link.send_image(data, node_id, show_progress)
            return "sent"

        self.tasks.put(send_task)


def main():
    root = tk.Tk()
    InjectorWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
