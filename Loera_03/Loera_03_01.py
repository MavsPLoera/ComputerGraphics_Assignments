# Loera, Preston
# 1001_889_535
# 2024_10_19
# Assignment_03_01

import numpy as np
import tkinter as tk
from tkinter import ttk
from tkinter import simpledialog, filedialog

#Region codes for clipping.
INSIDE = 0b000000
FRONT = 0b000001
BEHIND = 0b000010
LEFT = 0b000100
RIGHT = 0b001000
BELOW = 0b010000
ABOVE = 0b100000

class cl_world:
    worldCoords = np.array([[-1, -1, 1, 1]], dtype=float)
    viewportCoord = np.array([], dtype=float)
    faces = np.array([], dtype=int)
    verticies = np.array([], dtype=float)
    cameras = []

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

        '''Some functionality for Assignment 3'''
        ################### Fly Camera(s) ###################
        self.camera_panel = cl_camera_panel(self)

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

        #assume we have data already loaded if there is data in world coord
        if(np.size(cl_world.verticies) != 0):
            cl_world.faces = np.array([], dtype=int)
            cl_world.verticies = np.array([], dtype=float)

        for line in fileLines: #Set fileData to loaded vectices and faces (Can probably change this into verticies and faces arrays)
            line = line.strip().split()
            if(line == []):
                continue
        
            #Load in verticies and faces.
            match line[0]:
                case 'v':
                    cl_world.verticies = np.append(cl_world.verticies, [float(line[1]),float(line[2]), float(line[3]),1])
                case 'f':
                    cl_world.faces = np.append(cl_world.faces, [int(line[1]),int(line[2]),int(line[3])])

        cl_world.verticies = cl_world.verticies.reshape(-1, 4)
        cl_world.faces = cl_world.faces.reshape(-1, 3)

    def draw_objects(self,event=None):
        self.canvas.delete('all') #Clear canvas before drawing
        if(cl_world.verticies.size == 0):
            result = self.clipLine(.5,.5,-.5,1.5,1.5,1.5)
            print(result != False)
            return

        #Apply loaded in verticies to each composite camera matrix.
        #Each camera has a copy of the verticies that are loaded in. so they dont have to be realligned every loop for each camera cordinate system.
        for camera in cl_world.cameras:
            camera.applyComposite()

        #Draw points for every camera that is currently loaded
        for camera in cl_world.cameras:
            vminX = int(self.canvas.winfo_width() * camera.viewPort[0])
            vminY = int(self.canvas.winfo_height() * camera.viewPort[1])
            vmaxX = int(self.canvas.winfo_width() * camera.viewPort[2])
            vmaxY = int(self.canvas.winfo_height() * camera.viewPort[3])

            self.canvas.create_text(vminX + .80, vminY, anchor='nw', text=camera.name, fill="black", font=("Arial", 10))

            self.canvas.create_rectangle(vminX, vminY, vmaxX, vmaxY)

            #Get viewport to window ratio
            Sx = (vmaxX - vminX) / (cl_world.worldCoords[0][2] - cl_world.worldCoords[0][0]) #(Xvmax−Xvmin)/(Xwmax−Xwmin)
            Sy = (vmaxY - vminY) / (cl_world.worldCoords[0][3] - cl_world.worldCoords[0][1]) #(Yvmax−Yvmin)/(Ywmax−Ywmin)

            
            for face in cl_world.faces:
                x0, y0, z0, w0 = camera.verticies[face[0] - 1]
                x1, y1, z1, w1 = camera.verticies[face[1] - 1]
                x2, y2, z2, w2 = camera.verticies[face[2] - 1]

                line1 = self.clipLine(x0, y0, z0, x1, y1, z1)
                if(line1 != False):
                    self.canvas.create_line(self.calculateX(line1[0], Sx, vminX), self.calculateY(line1[1], Sy, vminY), 
                                            self.calculateX(line1[3], Sx, vminX), self.calculateY(line1[4], Sy, vminY))
                
                line2 = self.clipLine(x1, y1, z1, x2, y2, z2)
                if(line2 != False):
                    self.canvas.create_line(self.calculateX(line2[0], Sx, vminX), self.calculateY(line2[1], Sy, vminY), 
                                            self.calculateX(line2[3], Sx, vminX), self.calculateY(line2[4], Sy, vminY))

                line3 = self.clipLine(x2, y2, z2, x1, y1, z1)
                if(line3 != False):
                    self.canvas.create_line(self.calculateX(line3[0], Sx, vminX), self.calculateY(line3[1], Sy, vminY), 
                                            self.calculateX(line3[3], Sx, vminX), self.calculateY(line3[4], Sy, vminY))
                                            
            self.canvas.update()     
            self.message_label.config(text="Object is drawn")

    #Determine if the line being given should be drawn to canvas
    def clipLine(self, x0, y0, z0, x1, y1, z1):

        #Get location codes of both points passed in
        code1 = self.getPointLocationCode(x0, y0, z0)
        code2 = self.getPointLocationCode(x1, y1, z1)

        acceptLine = False

        while True:
            if (code1 | code2) == 0:  # Both points inside viewvolume (ACCEPT)
                acceptLine = True
                break
            elif (code1 & code2) != 0:  # Both points outside viewvolume (REJECT)
                break
            else: # Points are in different regions calculate intersection with one of the bounding planes

                #Select a point that is outside viewvolume. If both are outside select code that is greater in binary value.
                if code2 > code1:
                    outsideCode = code2
                else:
                    outsideCode = code1
                
                # Calculate intersection of point outside viewvolume. Need to keep in mind of the denominator being 0. If it is, we will calculate the next plane.
                if outsideCode & FRONT:
                    if z1 != z0: 
                        t = (1 - z0) / (z1 - z0)
                    else:
                        break 
                elif outsideCode & BEHIND:
                    if z1 != z0:
                        t = (-z0) / (z1 - z0)
                    else:
                        break
                elif outsideCode & LEFT:
                    if x1 != x0:
                        t = ((-1) - x0) / (x1 - x0)
                    else:
                        break
                elif outsideCode & RIGHT:
                    if x1 != x0:
                        t = (1 - x0) / (x1 - x0)
                    else:
                        break
                elif outsideCode & BELOW:
                    if y1 != y0:
                        t = ((-1) - y0) / (y1 - y0)
                    else:
                        break
                elif outsideCode & ABOVE:
                    if y1 != y0:
                        t = (1 - y0) / (y1 - y0)
                    else:
                        break

                # Calculate new line with intersection point
                xt = x0 + (t * (x1 - x0))
                yt = y0 + (t * (y1 - y0))
                zt = z0 + (t * (z1 - z0))

                #Apply new intersection to whatever outside point was selected.
                if outsideCode == code1:  # Point 1 is outside
                    x0, y0, z0 = xt, yt, zt
                    code1 = self.getPointLocationCode(x0, y0, z0)
                else:  # Point 2 is outside
                    x1, y1, z1 = xt, yt, zt
                    code2 = self.getPointLocationCode(x1, y1, z1)

        if (acceptLine == True):
            return (x0, y0, z0, x1, y1, z1)

        return False

    #Get bit code for point relative to view volume with bounds x = 1, x = -1, y = 1, y = -1, z = 0, z = 1
    def getPointLocationCode(self, x, y, z):
        code = INSIDE

        #Check x point relative to clipping region
        if(x < -1): 
            code |= LEFT
        elif(x > 1):
            code |= RIGHT

        #Check y point relative to clipping region
        if(y < -1):
            code |= BELOW
        elif(y > 1):
            code |= ABOVE

        #Check z point relative to clipping region
        if(z < 0):
            code |= BEHIND
        elif(z > 1):
            code |= FRONT      

        return code  

    def calculateX(self, xCoord, Sx, vminX):
        dX = (xCoord - cl_world.worldCoords[0][0])
        dPrimeX = dX * Sx
        xPrime = vminX + int(dPrimeX)
        return xPrime
    
    def calculateY(self, yCoord, Sy, vminY):
        dY = (cl_world.worldCoords[0][3] - yCoord)
        dPrimeY = dY * Sy
        yPrime = vminY + int(dPrimeY)
        return yPrime
    
    def draw_button_clicked(self):
        self.draw_objects()

    def canvas_resized(self,event=None):
        if self.canvas.find_all():
            self.draw_objects(event)

'''When cameras are laoded in from cameras.txt we will populate list full of cameras.'''

################### Camera ###################
class camera:
    verticies = np.array([], dtype=float)

    def __init__(self, name, type, vrp, vpn, vup, prp, viewVol, viewPort):
        self.name = name
        self.type = type
        self.vrp = vrp
        self.vpn = vpn
        self.vup = vup
        self.prp = prp
        self.viewVol = viewVol
        self.viewPort = viewPort
        self.composite = self.calculateComposite()

    #Applies composite matrix to the copy of the cameras points.
    def applyComposite(self):
        camera.verticies = np.copy(cl_world.verticies)

        for i, point in enumerate(camera.verticies):
            camera.verticies[i] = self.composite.dot(point)

    #Just used for debugging not really used in program as of now.
    def printCameraInfo(self):
        print("Camera name: \n", self.name)
        print("Camera type: \n", self.type)
        print("Camera vrp: \n", self.vrp)
        print("Camera vpn: \n", self.vpn)
        print("Camera vup: \n",self.vup)
        print("Camera prp: \n",self.prp)
        print("Camera viewVolume: \n",self.viewVol)
        print("Camera viewPort: \n",self.viewPort)
        print("Camera composite: \n", self.composite)

    #Calculate the composite matrix of the provided camera values. (Most likely will need to change the function depending on camera type.)
    def calculateComposite(self):

        #Step 1.) Translate VRP to origin
        vrpTranslation = np.array([[1, 0, 0, np.negative(self.vrp[0])],
                                [0, 1, 0, np.negative(self.vrp[1])],
                                [0, 0, 1, np.negative(self.vrp[2])],
                                [0, 0, 0, 1]])

        #Step 2.) Rotate VPN to lie on XZ plane.
        a,b,c,w = np.ravel(self.vpn)
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
        vupP = Rx.dot(self.vup)
        vpnP = Rx.dot(self.vpn)
        CM = Rx.dot(vrpTranslation)

        #Step 3.) Align VPN with z-axis
        a,b,c,w = np.ravel(vpnP)
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
        CM = Ry.dot(CM)
        vupP = Ry.dot(vupP)

        #Step 4.) Align VUP with y-axis
        a,b,c,w = np.ravel(vupP)
        temp_denom = np.sqrt((a*a)+(b*b))
        if temp_denom==0:
            Rz = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        else:
            Rz = np.array([[b/temp_denom,-a/temp_denom,0,0],
                        [a/temp_denom,b/temp_denom,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        CM = Rz.dot(CM)
        print(CM)

        #Get DOP (DOP = CW - PRP)
        CW = np.array([((self.viewVol[1] + self.viewVol[0])/2) - self.prp[0], ((self.viewVol[3] + self.viewVol[2])/2) - self.prp[1], 0 - self.prp[3]])
        #print(CW[0]/self.prp[2])
        #print(CW[1]/self.prp[2])

        #Step 5.) Shear DOP to align with vpn
        Sh = np.array([[1, 0, CW[0]/self.prp[2], 0],
                       [0, 1, CW[1]/self.prp[2], 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1],])
        CM = Sh.dot(CM)

        #Step 6.) Translate u,v,n to 2x2 cube
        dx = np.negative(self.viewVol[0] + self.viewVol[1])/2
        dy = np.negative(self.viewVol[2] + self.viewVol[3])/2
        dz = np.negative(self.viewVol[4])

        viewVolTranslation = np.array([[1, 0, 0, dx],
                                [0, 1, 0, dy],
                                [0, 0, 1, dz],
                                [0, 0, 0, 1]])
        CM = viewVolTranslation.dot(CM)

        #Step 7.) Scale
        sx = 1/(self.viewVol[1] + dx)
        sy = 1/(self.viewVol[3] + dy)
        sz = 1/(self.viewVol[5] + dz)

        S = np.array([[sx, 0, 0, 0],
                      [0, sy, 0, 0],
                      [0, 0, sz, 0],
                      [0, 0, 0, 1]])
        CM = S.dot(CM)
        print(CM)

        return CM

'''Defines widgets and fucntionality of TopLevel window for the fly camera.'''

################### Camera Functionality ###################
class cl_camera_panel:
    def __init__(self, master):
        self.master = master
        
        self.camera_frame = tk.Frame(master.root)
        self.camera_frame.pack(side=tk.TOP, fill=tk.X)

        self.defineCameras()

        self.flyCamera_button = tk.Button(self.camera_frame, text="Fly Camera", command = self.flyCameraWindow)
        self.flyCamera_button.pack(side=tk.LEFT)

    #Might need to do some checking if the filelines are blank for default values.
    def defineCameras(self):
        cameraFile = open("cameras.txt", 'r')
        cameraFileLines = cameraFile.readlines()

        indexs = []
        for i , line in enumerate(cameraFileLines):
            line.strip().split()
            if(line == 'c\n'):
                indexs.append(i)

        '''
        Get indexs by reading the lines of the file to find where "c" is since we know the file format should be the same we can + to the index to get certain things to define our camera(s)
        If the parameters in the line are not equal the amount we expecting we. send the default values to the camera constructor.
        '''
        for i in indexs:
            #print(cameraFileLines[i + 1],cameraFileLines[i + 2],cameraFileLines[i + 3],cameraFileLines[i + 4], cameraFileLines[i + 5],cameraFileLines[i + 6],cameraFileLines[i + 7],cameraFileLines[i + 8]) #For Debugging

            #Take first and second line after the c and define the name. takes the "c camera_name\n" removes the new line and makes new string after the 2nd character "camera_name". Same for camera type
            cameraName = cameraFileLines[i + 1].strip()[2:]
            cameraType = cameraFileLines[i + 2].strip()[2:]
            if(cameraName == ""):
                cameraType = "parallel"

            vrp = np.array([0, 0, 0, 1], dtype=float)
            vpn = np.array([0, 0, 1, 1], dtype=float)
            vup = np.array([0, 1, 0, 1], dtype=float)
            prp = np.array([0, 0, 1, 1], dtype=float)
            viewVolume = np.array([-1, 1, -1, 1, -1, 1], dtype=float)
            viewPort = np.array([0.1, 0.1, 0.4, 0.4], dtype=float)

            #Rest of the lines after the first 2 are similar in the steps we take.
            #Strip and split the line based on spaces to be able to create a list from the line. 
            #We then convert that list into a np.array to make it easy for matrix manipulation.
            vrpLine = cameraFileLines[i + 3]
            vrpLine = vrpLine.strip().split()
            if len(vrpLine) == 4:
                vrp = np.array([vrpLine[1], vrpLine[2], vrpLine[3],1], dtype = float)
            
            vpnLine = cameraFileLines[i + 4]
            vpnLine = vpnLine.strip().split()
            if len(vpnLine) == 4:
                vpn = np.array([vpnLine[1], vpnLine[2], vpnLine[3],1], dtype = float)

            vupLine = cameraFileLines[i + 5]
            vupLine = vupLine.strip().split()
            if len(vupLine) == 4:
                vup = np.array([vupLine[1], vupLine[2], vupLine[3],1], dtype = float)

            prpLine = cameraFileLines[i + 6]
            prpLine = prpLine.strip().split()
            if len(prpLine) == 4:
                prp = np.array([prpLine[1], prpLine[2], prpLine[3],1], dtype = float)

            viewVolumeLine = cameraFileLines[i + 7]
            viewVolumeLine = viewVolumeLine.strip().split()
            if len(viewVolumeLine) == 7:
                viewVolume = np.array([float(viewVolumeLine[1]), float(viewVolumeLine[2]), float(viewVolumeLine[3]), float(viewVolumeLine[4]), float(viewVolumeLine[5]), float(viewVolumeLine[6])], dtype=float)

            viewPortLine = cameraFileLines[i + 8]
            viewPortLine = viewPortLine.strip().split()
            if len(viewPortLine) == 5:
                viewPort = np.array([viewPortLine[1], viewPortLine[2], viewPortLine[3], viewPortLine[4]], dtype = float)

            cl_world.cameras.append(camera(cameraName, cameraType, vrp, vpn, vup, prp, viewVolume, viewPort))

    def flyCameraWindow(self):
        self.cameraWindow = tk.Toplevel(self.master.root)
        self.cameraWindow.title('Select Camera')
        self.cameraWindow.resizable(False, False)
        self.cameraWindow.geometry('750x25')

        self.cameraFrame = tk.Frame(self.cameraWindow)
        self.cameraFrame.pack(side=tk.TOP, fill=tk.X)

        self.selectLabel = tk.Label(self.cameraFrame, text = "Select camera to fly:")
        self.selectLabel.pack(side=tk.LEFT)

        cameraNames = []
        for camera in cl_world.cameras:
            cameraNames.append(camera.name)

        self.camera_string = tk.StringVar(value = cameraNames[0])
        self.dropDown = ttk.Combobox(self.cameraFrame, textvariable = self.camera_string)
        self.dropDown['values'] = cameraNames
        self.dropDown.pack(side=tk.LEFT)
        self.dropDown.bind('<<ComboboxSelected>>', self.getCurCameraVRP)

        self.CurrentVRP_Label = tk.Label(self.cameraFrame, text = "Current VRP([x,y,z]): ")
        self.CurrentVRP_Label.pack(side=tk.LEFT)
        self.VRP_Entry = tk.Entry(self.cameraFrame, textvariable = self.CurrentVRP_Label, width = 10)
        self.getCurCameraVRP() #Gets called once on the window creation to set the entry to have the value of the first loaded camera
        self.VRP_Entry.pack(side=tk.LEFT)

        self.VRP2_Label = tk.Label(self.cameraFrame, text = "VRP 2([x,y,z]): ")
        self.VRP2_Label.pack(side=tk.LEFT)
        self.VRP2_Entry = tk.Entry(self.cameraFrame, textvariable = self.VRP2_Label, width = 10)
        self.VRP2_Entry.insert(0, '[1.0,1.0,1.0]')
        self.VRP2_Entry.pack(side=tk.LEFT)

        self.Fly_Steps_Label = tk.Label(self.cameraFrame, text = "Steps: ")
        self.Fly_Steps_Label.pack(side=tk.LEFT)
        self.Fly_Steps_spinbox = tk.Spinbox(self.cameraFrame, from_ = 1, to = 100, repeatdelay=500, repeatinterval=100, width = 5)
        self.Fly_Steps_spinbox.pack(side = tk.LEFT)

        #Need to add command for fly button.
        #Also need to swap VRPs when done using update VRP Entry. Might need to update that mehtod or make another for the VRP2 entry
        self.Fly_Button = tk.Button(self.cameraFrame, text="Fly", width = 10, command = self.flyCamera)
        self.Fly_Button.pack(side=tk.LEFT)

    #Sets vrp value on swap for camera might need to do more with this later.
    def getCurCameraVRP(self, event=None):
        currentSelection = self.dropDown.current()
        currentCameraVRP = cl_world.cameras[currentSelection].vrp
        vrp = f"[{currentCameraVRP[0]},{currentCameraVRP[1]},{currentCameraVRP[2]}]"
        self.VRP_Entry.delete(0, tk.END)
        self.VRP_Entry.insert(0, vrp)

    #Swaps the VRP for the entries when Fly is clicked. Can probably be implemented with moving the VRP as well.
    def flyCamera(self):
        cameraToFly = self.dropDown.current()
        prevVRP = np.array(list(self.VRP_Entry.get().strip('][').split(',')), dtype=float)
        newVRP = np.array(list(self.VRP2_Entry.get().strip('][').split(',')), dtype=float) 
        steps = int(self.Fly_Steps_spinbox.get())

        dX = (newVRP[0] - prevVRP[0]) / steps
        dY = (newVRP[1] - prevVRP[1]) / steps
        dZ = (newVRP[2] - prevVRP[2]) / steps

        for steps in range(steps):
            cl_world.cameras[cameraToFly].vrp[0] = cl_world.cameras[cameraToFly].vrp[0] + dX
            cl_world.cameras[cameraToFly].vrp[1] = cl_world.cameras[cameraToFly].vrp[1] + dY
            cl_world.cameras[cameraToFly].vrp[2] = cl_world.cameras[cameraToFly].vrp[2] + dZ
            cl_world.cameras[cameraToFly].composite = cl_world.cameras[cameraToFly].calculateComposite()
            self.master.draw_objects()

        #Update new cameraVRP in fly window
        self.getCurCameraVRP()

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
        self.T_Entry.insert(0, '[0.1,0.1,0.1]') #takes in two arguments the text positon and the text itself
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