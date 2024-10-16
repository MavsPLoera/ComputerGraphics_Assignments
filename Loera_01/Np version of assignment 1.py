import numpy as np
import tkinter as tk
from tkinter import filedialog

class cl_world:
    def __init__(self):
        self.worldCoords = np.array([])
        self.viewportCoord = np.array([])
        self.faces = []
        self.verticies = []
        
        self.root = tk.Tk()
        self.root.title("Resizable Window")
        self.root.geometry("400x600")
        self.root.resizable(True, True)

        self.top_frame = tk.Frame(self.root)
        self.top_frame.pack(side=tk.TOP, fill=tk.X)
        self.browse_button = tk.Button(self.top_frame, text="Browse", fg="blue", command=self.browse_file_clicked)
        self.browse_button.pack(side=tk.LEFT)
        self.draw_button = tk.Button(self.top_frame, text="Draw", command=self.draw_objects)
        self.draw_button.pack(side=tk.LEFT, padx=10, pady=10)

        self.canvas = tk.Canvas(self.root, bg="light goldenrod")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self.canvas_resized)

        self.bottom_frame = tk.Frame(self.root)
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.message_label = tk.Label(self.bottom_frame, text="")
        self.message_label.pack(padx=10, pady=10)

    def browse_file_clicked(self):
        self.file_path = filedialog.askopenfilename(filetypes=[("All Files", "*"), ("Text Files", "*.txt")])
        self.message_label.config(text=self.file_path)
        self.load_file(self.file_path)

    def canvas_resized(self, event=None):
        if self.canvas.find_all():
            self.draw_objects()

    def load_file(self, filename):
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip().split()
                if not line:
                    continue
                match line[0]:
                    case 'v':
                        self.verticies.append([float(line[1]), float(line[2])])  # Store as 2D points
                    case 'f':
                        self.faces.append([int(line[1]), int(line[2]), int(line[3])])
                    case 'w':
                        self.worldCoords = np.array([float(x) for x in line[1:5]])
                    case 's':
                        self.viewportCoord = np.array([float(x) for x in line[1:5]])

        self.verticies = np.array(self.verticies)
        self.faces = np.array(self.faces)
        print(self.verticies)

    def draw_objects(self):
        self.canvas.delete('all')
        if self.viewportCoord.size == 0:
            print("No viewport coordinates defined.")
            return

        vminX = int(self.canvas.winfo_width() * self.viewportCoord[0])
        vminY = int(self.canvas.winfo_height() * self.viewportCoord[1])
        vmaxX = int(self.canvas.winfo_width() * self.viewportCoord[2])
        vmaxY = int(self.canvas.winfo_height() * self.viewportCoord[3])

        self.canvas.create_rectangle(vminX, vminY, vmaxX, vmaxY)

        if self.worldCoords.size == 0:
            print("No world coordinates defined.")
            return

        Sx = (vmaxX - vminX) / (self.worldCoords[2] - self.worldCoords[0])
        Sy = (vmaxY - vminY) / (self.worldCoords[3] - self.worldCoords[1])

        for face in self.faces:
            coords = []
            for vertex_index in face - 1:  # Adjusting for zero-indexing
                x = self.verticies[vertex_index, 0]
                y = self.verticies[vertex_index, 1]
                x_prime = vminX + int((x - self.worldCoords[0]) * Sx)
                y_prime = vminY + int((self.worldCoords[3] - y) * Sy)
                coords.extend((x_prime, y_prime))
            self.canvas.create_polygon(coords, outline='black', fill='')

        self.message_label.config(text="Objects drawn.")

# Run the tkinter main loop
if __name__ == "__main__":
    world = cl_world()
    world.root.mainloop()
