# Loera, Preston
# 1001_889_535
# 2024_09_06
# Assignment_01_01

import numpy as np
import tkinter as tk
from tkinter import simpledialog, filedialog

class cl_world:
    worldCoords = []
    viewportCoord = []
    faces = []
    verticies = []

    def __init__(self):
        ################## Main Window ########################
        # Initialize the main window
        self.root = tk.Tk()
        self.root.title("Resizable Window")
        # Set the window gemetry (Size) and Make it resizable
        self.root.geometry("400x600")
        self.root.resizable(True, True)
        ################### Top Pnael ##########################
        # Create a top frame for the button
        self.top_frame = tk.Frame(self.root)
        self.top_frame.pack(side=tk.TOP, fill=tk.X)
        # Create a button in the top panel
        self.brwose_button = tk.Button(self.top_frame, text="Browse", fg="blue", command=self.browse_file_clicked)
        self.brwose_button.pack(side=tk.LEFT)
        self.draw_button = tk.Button(self.top_frame, text="Draw", command=self.draw_button_clicked)
        self.draw_button.pack(side=tk.LEFT, padx=10, pady=10)
        ################### Canvas #############################
        # Create a canvas to draw on
        self.canvas = tk.Canvas(self.root, bg="light goldenrod")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        # Bind the resize event to redraw the canvas when window is resized
        self.canvas.bind("<Configure>", self.canvas_resized)
        #################### Bottom Panel #######################
        # Create a bottom frame for displaying messages
        self.bottom_frame = tk.Frame(self.root)
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)
        # Create a lebel for showing messages
        self.message_label = tk.Label(self.bottom_frame, text="")
        self.message_label.pack(padx=10, pady=10)

    def browse_file_clicked(self):
        self.file_path = tk.filedialog.askopenfilename(filetypes=[("allfiles", "*"), ("pythonfiles", "*.txt")])
        self.message_label.config(text=self.file_path)
        self.load_file(self.file_path)

    def draw_button_clicked(self):
        self.draw_objects()

    def canvas_resized(self,event=None):
        if self.canvas.find_all():
            self.draw_objects(event)

    def load_file(self,filename):
        file = open(filename, 'r')
        fileLines = file.readlines() 

        if(cl_world.worldCoords != []): #assume we have data already loaded if there is data in world coord
            del cl_world.worldCoords[:]
            del cl_world.viewportCoord[:]
            del cl_world.faces[:]
            del cl_world.verticies[:]

        for line in fileLines: #Set fileData to loaded vectices and faces (Can probably change this into verticies and faces arrays)
            line = line.strip().split()
            if(line == []):
                continue
        
            #Check first component of line. Add to respect list based on case (IGNORE Z).
            match line[0]:
                case 'v':
                    cl_world.verticies.append((float(line[1]),float(line[2])))
                case 'f':
                    cl_world.faces.append((int(line[1]),int(line[2]),int(line[3])))
                case 'w':
                    cl_world.worldCoords.append(float(line[1]))
                    cl_world.worldCoords.append(float(line[2]))
                    cl_world.worldCoords.append(float(line[3]))
                    cl_world.worldCoords.append(float(line[4]))
                case 's':
                    cl_world.viewportCoord.append(float(line[1]))
                    cl_world.viewportCoord.append(float(line[2]))
                    cl_world.viewportCoord.append(float(line[3]))
                    cl_world.viewportCoord.append(float(line[4]))

    def draw_objects(self,event=None):
        self.canvas.delete('all') #Clear canvas before drawing
        if(cl_world.viewportCoord == []):
            print("Data not loaded, no objects drawn")
            return

        #Get viewport ratio to screen width and height
        vminX = int(self.canvas.winfo_width() * cl_world.viewportCoord[0])
        vminY = int(self.canvas.winfo_height() * cl_world.viewportCoord[1])
        vmaxX = int(self.canvas.winfo_width() * cl_world.viewportCoord[2])
        vmaxY = int(self.canvas.winfo_height() * cl_world.viewportCoord[3])

        self.canvas.create_rectangle(vminX, vminY, vmaxX, vmaxY)

        #Get viewport to window ratio
        Sx = (vmaxX - vminX) / (cl_world.worldCoords[2] - cl_world.worldCoords[0]) #(Xvmax−Xvmin)/(Xwmax−Xwmin)
        Sy = (vmaxY - vminY) / (cl_world.worldCoords[3] - cl_world.worldCoords[1]) #(Yvmax−Yvmin)/(Ywmax−Ywmin)

        for face in cl_world.faces:
             self.canvas.create_polygon(self.calculateX(cl_world.verticies[face[0] - 1][0], Sx, vminX), self.calculateY(cl_world.verticies[face[0] - 1][1], Sy, vminY),
                                        self.calculateX(cl_world.verticies[face[1] - 1][0], Sx, vminX), self.calculateY(cl_world.verticies[face[1] - 1][1], Sy, vminY),
                                        self.calculateX(cl_world.verticies[face[2] - 1][0], Sx, vminX), self.calculateY(cl_world.verticies[face[2] - 1][1], Sy, vminY), 
                                        outline = 'black', fill = '')
        self.message_label.config(text="Object is drawn")

    def calculateX(self, xCoord, Sx, vminX):
        dX = (xCoord - cl_world.worldCoords[0])
        dPrimeX = dX * Sx
        xPrime = vminX + int(dPrimeX)
        return xPrime
    
    def calculateY(self, yCoord, Sy, vminY):
        dY = (cl_world.worldCoords[3] - yCoord)
        dPrimeY = dY * Sy
        yPrime = vminY + int(dPrimeY)
        return yPrime
    
    def draw_button_clicked(self):
        self.draw_objects()

    def canvas_resized(self,event=None):
        if self.canvas.find_all():
            self.draw_objects(event)

# Run the tkinter main loop
world=cl_world()
world.root.mainloop()
