# Loera, Preston
# 1001_889_535
# 2024_10_01
# Assignment_02_01

import numpy as np
import tkinter as tk
from tkinter import simpledialog, filedialog

class cl_world:
    worldCoords = np.array([], dtype=float)
    viewportCoord = np.array([], dtype=float)
    faces = np.array([], dtype=int)
    verticies = np.array([], dtype=float)

    def __init__(self):
        ################## Main Window ########################
        # Initialize the main window
        self.root = tk.Tk()
        self.root.title("Resizable Window")
        # Set the window gemetry (Size) and Make it resizable
        self.root.geometry("600x700")
        self.root.resizable(True, True)
        ################### Top Pnael ##########################
        # Create a top frame for the button
        self.top_frame = tk.Frame(self.root)
        self.top_frame.pack(side=tk.TOP, fill=tk.X)
        # Create a button in the top panel
        self.brwose_button = tk.Button(self.top_frame, text="Browse", fg="blue", command=self.browse_file_clicked)
        self.brwose_button.pack(side=tk.LEFT)

        self.draw_button = tk.Button(self.top_frame, text="Draw", command=self.draw_button_clicked)
        self.draw_button.pack(side=tk.RIGHT, padx=10, pady=10)

        '''New panels that were added for Assignment 2 functionality.'''
        ################### Rotation, Scale, and Translation ###################
        self.rotation_panel = cl_rotation_panel(self)
        self.scale_panel = cl_scale_panel(self)
        self.tranlation_panel = cl_translation_panel(self)

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

        if(np.size(cl_world.worldCoords) != 0): #assume we have data already loaded if there is data in world coord
            cl_world.worldCoords = np.array([], dtype=float)
            cl_world.viewportCoord = np.array([], dtype=float)
            cl_world.faces = np.array([], dtype=int)
            cl_world.verticies = np.array([], dtype=float)

        for line in fileLines: #Set fileData to loaded vectices and faces (Can probably change this into verticies and faces arrays)
            line = line.strip().split()
            if(line == []):
                continue
        
            #Check first component of line. Add to respect list based on case (IGNORE Z).
            match line[0]:
                case 'v':
                    cl_world.verticies = np.append(cl_world.verticies, [float(line[1]),float(line[2]), float(line[3]),1])
                case 'f':
                    cl_world.faces = np.append(cl_world.faces, [int(line[1]),int(line[2]),int(line[3])])
                case 'w':
                    cl_world.worldCoords = np.append(cl_world.worldCoords, float(line[1]))
                    cl_world.worldCoords = np.append(cl_world.worldCoords, float(line[2]))
                    cl_world.worldCoords = np.append(cl_world.worldCoords, float(line[3]))
                    cl_world.worldCoords = np.append(cl_world.worldCoords, float(line[4]))
                case 's':
                    cl_world.viewportCoord = np.append(cl_world.viewportCoord, float(line[1]))
                    cl_world.viewportCoord = np.append(cl_world.viewportCoord, float(line[2]))
                    cl_world.viewportCoord = np.append(cl_world.viewportCoord, float(line[3]))
                    cl_world.viewportCoord = np.append(cl_world.viewportCoord, float(line[4]))

        cl_world.verticies = cl_world.verticies.reshape(-1, 4)
        cl_world.faces = cl_world.faces.reshape(-1, 3)

    def draw_objects(self,event=None):
        self.canvas.delete('all') #Clear canvas before drawing
        if(cl_world.viewportCoord.size == 0):
                #print("Size == 0")
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
        self.canvas.update()     
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

'''Note: Rotate, Scale, and Translation are in their own classes to make it more readable about how each function is getting values from their respective widgets.'''

################### Rotation ###################
class cl_rotation_panel:
    def __init__(self, master):
        self.master = master
        self.Rotation_Frame = tk.Frame(master.root)
        self.Rotation_Frame.pack(side=tk.TOP, fill=tk.X)

        self.Line_AB_Text = tk.Label(self.Rotation_Frame, text = "Line AB")
        self.Line_AB_Text.pack(side=tk.LEFT)

        self.A_Entry_Label = tk.Label(self.Rotation_Frame, text = "A: ")
        self.A_Entry_Label.pack(side=tk.LEFT)
        self.A_Entry = tk.Entry(self.Rotation_Frame, textvariable = self.A_Entry_Label, width = 10)
        self.A_Entry.insert(0, '[0.0,0.0,0.0]') #takes in two arguments the text positon and the text itself
        self.A_Entry.pack(side=tk.LEFT)

        self.B_Entry_Label = tk.Label(self.Rotation_Frame, text = "B: ")
        self.B_Entry_Label.pack(side=tk.LEFT)
        self.B_Entry = tk.Entry(self.Rotation_Frame, textvariable = self.B_Entry_Label, width = 10)
        self.B_Entry.insert(0, '[1.0,1.0,1.0]') #takes in two arguments the text positon and the text itself
        self.B_Entry.pack(side=tk.LEFT)

        self.D_Label = tk.Label(self.Rotation_Frame, text = "Degree: ")
        self.D_Label.pack(side=tk.LEFT)
        self.Degree_spinbox = tk.Spinbox(self.Rotation_Frame, from_ = 0, to = 100, repeatdelay=500, repeatinterval=100, width = 5)
        self.Degree_spinbox.pack(side = tk.LEFT)

        self.Roation_Steps_Label = tk.Label(self.Rotation_Frame, text = "Steps: ")
        self.Roation_Steps_Label.pack(side=tk.LEFT)
        self.Rotation_Steps_spinbox = tk.Spinbox(self.Rotation_Frame, from_ = 1, to = 100, repeatdelay=500, repeatinterval=100, width = 5)
        self.Rotation_Steps_spinbox.pack(side = tk.LEFT)

        self.Rotate_Button = tk.Button(self.Rotation_Frame, text="Rotate", command = self.rotate_button_clicked)
        self.Rotate_Button.pack(side=tk.LEFT)

    def rotate_button_clicked(self):
        #Get values from respective, entries and spinboxes
        print("Rotation")
        if(np.size(cl_world.worldCoords) == 0):
            print("no points to print")
            return

        pointA = np.array(list(self.A_Entry.get().strip('][').split(',')), dtype=float)
        pointB = np.array(list(self.B_Entry.get().strip('][').split(',')), dtype=float) 
        degrees = int(self.Degree_spinbox.get())
        steps = int(self.Rotation_Steps_spinbox.get())

        #Get DOP from line AB
        dir = np.subtract(pointB, pointA)
        #print(dir)
        dir = np.append(dir, 1)

        #Steps:
        #1.) Translate point to origin
        tranlation_ToOrigin = np.array([[1,0,0,np.negative(pointA[0])],
                                        [0,1,0,np.negative(pointA[1])],
                                        [0,0,1,np.negative(pointA[2])],
                                        [0,0,0,1]])

        #2.) Move line AB to XZ plane
        a,b,c,w = np.ravel(dir)
        temp_denom = np.sqrt((b*b)+(c*c))
        if temp_denom==0:
            Rx = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        else:
            Rx = np.array([[1,0,0,0],
                        [0,c/temp_denom,-b/temp_denom,0],
                        [0,b/temp_denom,c/temp_denom,0],
                        [0,0,0,1]])
        dirp = Rx.dot(dir)
        CM = Rx.dot(tranlation_ToOrigin)

        #3.) Align line AB to z-plane
        a,b,c,w = np.ravel(dirp)
        temp_denom = np.sqrt((a*a)+(c*c))
        if temp_denom==0:
            Ry = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        else:
            Ry = np.array([[c/temp_denom,0,-a/temp_denom,0],
                        [0,1,0,0],
                        [a/temp_denom,0,c/temp_denom,0],
                        [0,0,0,1]])
        dirpp = Ry.dot(dirp)
        #print(dirpp)
        CM = Ry.dot(CM)

        #4.) Rotate around z axis
        radians = np.deg2rad(degrees/steps)
        rotation = np.array([[np.cos(radians), np.negative(np.sin(radians)), 0 ,0], 
                            [np.sin(radians), np.cos(radians), 0, 0],
                            [0,0,1,0],
                            [0,0,0,1]], dtype = float)
        CM = rotation.dot(CM)

        #5.) Invert Ry back
        invertRY = np.linalg.inv(Ry)
        CM = invertRY.dot(CM)

        #6.) Invert Rx back
        invertRX = np.linalg.inv(Rx)
        CM = invertRX.dot(CM)

        #7.) Translate back
        translation_Back = np.linalg.inv(tranlation_ToOrigin)
        CM = translation_Back.dot(CM)
        
        for steps in range(steps):
            for i, point in enumerate(cl_world.verticies):
                cl_world.verticies[i] = CM.dot(point)
            self.master.draw_objects()

################### Scale ###################
class cl_scale_panel:
    def __init__(self, master):
        self.master = master
        self.Scale_Frame = tk.Frame(master.root)
        self.Scale_Frame.pack(side=tk.TOP, fill=tk.X)

        self.Scale_Point_Text = tk.Label(self.Scale_Frame, text = "Scale about point:")
        self.Scale_Point_Text.pack(side=tk.LEFT)

        self.Point_Label = tk.Label(self.Scale_Frame)
        self.Point_Label.pack(side=tk.LEFT)
        self.Point_Entry = tk.Entry(self.Scale_Frame, width = 10)
        self.Point_Entry.insert(0, '[0.0,0.0,0.0]') #takes in two arguments the text positon and the text itself
        self.Point_Entry.pack(side=tk.LEFT)

        self.D_Label = tk.Label(self.Scale_Frame, text = "Scale:")
        self.D_Label.pack(side=tk.LEFT)

        self.Scale_Entry = tk.Entry(self.Scale_Frame, width = 10)
        self.Scale_Entry.insert(0, '[1.0,1.0,1.0]') #takes in two arguments the text positon and the text itself
        self.Scale_Entry.pack(side=tk.LEFT)

        self.Scale_Steps_Label = tk.Label(self.Scale_Frame, text = "Steps:")
        self.Scale_Steps_Label.pack(side=tk.LEFT)
        self.Scale_Steps_spinbox = tk.Spinbox(self.Scale_Frame, from_ = 1, to = 100, repeatdelay=500, repeatinterval=100, width = 5)
        self.Scale_Steps_spinbox.pack(side = tk.LEFT)

        self.Scale_Button = tk.Button(self.Scale_Frame, text="Scale", command = self.scale_button_clicked)
        self.Scale_Button.pack(side=tk.LEFT)

    def scale_button_clicked(self):
        #Get values from respective, entries and spinboxes
        print("Scale")
        if(np.size(cl_world.worldCoords) == 0):
            print("no points to print")
            return

        point = np.array(self.Point_Entry.get().strip('][').split(','), dtype=float)
        dx, dy, dz = np.ravel(point)
        steps = int(self.Scale_Steps_spinbox.get())
        scaleFactors = np.array(self.Scale_Entry.get().strip('][').split(','), dtype = float)
        Sx, Sy, Sz = np.ravel(scaleFactors)

        #Steps:
        # 1.) Translate point to origin
        # 2.) Scale it
        # 3.) Translate back
        translationToOrigin = np.array([[1,0,0,-(dx)],
                                        [0,1,0,-(dy)],
                                        [0,0,1,-(dz)],
                                        [0,0,0,1]])

        scale = np.array([[Sx,0,0,0],
                          [0,Sy,0,0],
                          [0,0,Sz,0],
                          [0,0,0,1]])
        
        translateBack = np.linalg.inv(translationToOrigin)

        #Get composite matrix
        CM = scale.dot(translationToOrigin)
        CM = translateBack.dot(CM)

        verticiesCopy = np.copy(cl_world.verticies)
        verticiesOriginal = np.copy(cl_world.verticies)
        for i, point in enumerate(cl_world.verticies):
                verticiesCopy[i] = CM.dot(point)

        #Perform this in N steps (FIX THIS)
        for step in range(steps):
            for i, point in enumerate(cl_world.verticies):
                cl_world.verticies[i][0] = ((verticiesCopy[i][0] - verticiesOriginal[i][0])/steps) + cl_world.verticies[i][0]
                cl_world.verticies[i][1] = ((verticiesCopy[i][1] - verticiesOriginal[i][1])/steps) + cl_world.verticies[i][1]
                cl_world.verticies[i][2] = ((verticiesCopy[i][2] - verticiesOriginal[i][2])/steps) + cl_world.verticies[i][2]
            self.master.draw_objects()

################### Translation ###################
class cl_translation_panel:
    def __init__(self, master):
        self.master = master
        self.Translation_Frame = tk.Frame(master.root)
        self.Translation_Frame.pack(side=tk.TOP, fill=tk.X)

        self.Translation_Point_Text = tk.Label(self.Translation_Frame, text = "Translation")
        self.Translation_Point_Text.pack(side=tk.LEFT)

        self.T_Entry_Label = tk.Label(self.Translation_Frame, text = "([dx, dy,dz]):")
        self.T_Entry_Label.pack(side=tk.LEFT)
        self.T_Entry = tk.Entry(self.Translation_Frame, width = 10)
        self.T_Entry.insert(0, '[0.0,0.0,0.0]') #takes in two arguments the text positon and the text itself
        self.T_Entry.pack(side=tk.LEFT)

        self.Translate_Steps_Label = tk.Label(self.Translation_Frame, text = "Steps: ")
        self.Translate_Steps_Label.pack(side=tk.LEFT)
        self.Translate_Steps_spinbox = tk.Spinbox(self.Translation_Frame, from_ = 1, to = 100, repeatdelay=500, repeatinterval=100, width = 5)
        self.Translate_Steps_spinbox.pack(side = tk.LEFT)

        self.Translate_Button = tk.Button(self.Translation_Frame, text="Translate", command = self.translation_button_clicked)
        self.Translate_Button.pack(side=tk.LEFT)

    def translation_button_clicked(self):
        #Get values from respective, entries and spinboxes
        print("Translation")
        if(np.size(cl_world.worldCoords) == 0):
            print("no points to print")
            return
        
        translation = np.array(self.T_Entry.get().strip('][').split(','), dtype = float)
        dx, dy, dz = np.ravel(translation)
        Nsteps = int(self.Translate_Steps_spinbox.get())

        #get value for each step
        x_step = float(dx / Nsteps)
        y_step = float(dy / Nsteps)
        z_step = float(dz / Nsteps)

        #Create translation matrix 
        translationMatrix = np.array([[1,0,0,x_step],
                                      [0,1,0,y_step],
                                      [0,0,1,z_step],
                                      [0,0,0,1]])

        # Perform steps N times
        for step in range(Nsteps):
            for i, point in enumerate(cl_world.verticies):
                cl_world.verticies[i] = translationMatrix.dot(point)
            self.master.draw_objects()
    
# Run the tkinter main loop
world=cl_world()
world.root.mainloop()
