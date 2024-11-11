# Loera, Preston
# 1001_889_535
# 2024_11_01
# Assignment_04_01

import numpy as np
import tkinter as tk
from tkinter import ttk
from tkinter import simpledialog, filedialog
from abc import ABC, abstractmethod

bezierResolution = 2 #Value is 2 by default until file is read
CONST_MAX_PATCHES = 100

#Region codes for clipping parallel and perspective. Same codes just different meaning for clipping.
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
    patches = np.empty((0, 16, 4), dtype=float)
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
        '''Assignment 5 functionality'''
        self.canvas.bind('<r>', self.r_key_pressed_callback)
        self.canvas.bind('<R>', self.R_key_pressed_callback)
        #################### Bottom Panel #######################
        # Create a bottom frame for displaying messages
        self.bottom_frame = tk.Frame(self.root)
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)
        # Create a lebel for showing messages
        self.message_label = tk.Label(self.bottom_frame, text="")
        self.message_label.pack(padx=10, pady=10)
        self.canvas.focus_set()

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

        temp = np.array([], dtype=float)
        pointCount = 0
        for line in fileLines: #Set fileData to loaded vectices and faces (Can probably change this into verticies and faces arrays)
            line = line.strip().split()
            if(line == []):
                continue
        
            #Load in verticies and faces.
            match line[0]:
                case 'n':
                    global bezierResolution
                    bezierResolution = int(line[1])
                case 'b':
                    temp = np.append(temp, [float(line[1]),float(line[2]),float(line[3]), 1]) #Making this homogeneous just in case
                    pointCount += 1

                    if(pointCount == 16):
                        temp = temp.reshape(-1, 4)
                        cl_world.patches = np.concatenate((cl_world.patches, temp[np.newaxis]), axis = 0)
                        temp = np.array([], dtype=float)
                        pointCount = 0
                case 'v':
                    cl_world.verticies = np.append(cl_world.verticies, [float(line[1]),float(line[2]), float(line[3]),1])
                case 'f':
                    cl_world.faces = np.append(cl_world.faces, [int(line[1]),int(line[2]),int(line[3])])

        cl_world.verticies = cl_world.verticies.reshape(-1, 4)
        cl_world.faces = cl_world.faces.reshape(-1, 3)

        print((cl_world.patches.size / 64))
        if((cl_world.patches.size / 64) > CONST_MAX_PATCHES):
            print("Patches loaded in exceed 100.")
            exit()

        file.close()

    # When 'r' is pressed, update the status label
    def r_key_pressed_callback(self, event):
        global bezierResolution
        if((bezierResolution - 1) < 2):
            bezierResolution = 2
        else:
            bezierResolution = bezierResolution - 1 
            self.draw_objects(event)

        print(bezierResolution)

    # When 'R' is pressed, update the status label
    def R_key_pressed_callback(self, event):
        global bezierResolution
        if((bezierResolution + 1) > 100):
            bezierResolution = 100
        else:
            bezierResolution = bezierResolution + 1
            self.draw_objects(event)

        print(bezierResolution) 

    def draw_objects(self,event=None):
        if((cl_world.verticies.size == 0) and (cl_world.patches.size == 0)):
            return

        self.canvas.delete('all') #Clear canvas before drawing

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

            Sx = (vmaxX - vminX) / (cl_world.worldCoords[0][2] - cl_world.worldCoords[0][0]) #(Xvmax−Xvmin)/(Xwmax−Xwmin)
            Sy = (vmaxY - vminY) / (cl_world.worldCoords[0][3] - cl_world.worldCoords[0][1]) #(Yvmax−Yvmin)/(Ywmax−Ywmin)

            self.canvas.create_text(vminX + .80, vminY, anchor='nw', text=camera.name, fill="black", font=("Arial", 10))
            self.canvas.create_rectangle(vminX, vminY, vmaxX, vmaxY)

            for face in cl_world.faces:
                x0, y0, z0, w0 = camera.verticies[face[0] - 1]
                x1, y1, z1, w1 = camera.verticies[face[1] - 1]
                x2, y2, z2, w2 = camera.verticies[face[2] - 1]

                line1 = camera.clipLine(x0, y0, z0, x1, y1, z1)
                if(line1 != False):
                    self.canvas.create_line(self.calculateX(line1[0], Sx, vminX), self.calculateY(line1[1], Sy, vminY), 
                                            self.calculateX(line1[3], Sx, vminX), self.calculateY(line1[4], Sy, vminY))
                
                line2 = camera.clipLine(x1, y1, z1, x2, y2, z2)
                if(line2 != False):
                    self.canvas.create_line(self.calculateX(line2[0], Sx, vminX), self.calculateY(line2[1], Sy, vminY), 
                                            self.calculateX(line2[3], Sx, vminX), self.calculateY(line2[4], Sy, vminY))

                line3 = camera.clipLine(x2, y2, z2, x0, y0, z0)
                if(line3 != False):
                    self.canvas.create_line(self.calculateX(line3[0], Sx, vminX), self.calculateY(line3[1], Sy, vminY), 
                                            self.calculateX(line3[3], Sx, vminX), self.calculateY(line3[4], Sy, vminY))  

            for patch in camera.patches:
                #Send points for a patch to calculate patch, get returned triangles and render them on canvas.
                triangles = self.calculatePatch(patch)

                for triangle in triangles:
                    x0, y0, z0 = triangle[0]
                    x1, y1, z1 = triangle[1]
                    x2, y2, z2  = triangle[2]

                    line1 = camera.clipLine(x0, y0, z0, x1, y1, z1)
                    if(line1 != False):
                        self.canvas.create_line(self.calculateX(line1[0], Sx, vminX), self.calculateY(line1[1], Sy, vminY), 
                                                self.calculateX(line1[3], Sx, vminX), self.calculateY(line1[4], Sy, vminY))
                    
                    line2 = camera.clipLine(x1, y1, z1, x2, y2, z2)
                    if(line2 != False):
                        self.canvas.create_line(self.calculateX(line2[0], Sx, vminX), self.calculateY(line2[1], Sy, vminY), 
                                                self.calculateX(line2[3], Sx, vminX), self.calculateY(line2[4], Sy, vminY))

                    line3 = camera.clipLine(x2, y2, z2, x0, y0, z0)
                    if(line3 != False):
                        self.canvas.create_line(self.calculateX(line3[0], Sx, vminX), self.calculateY(line3[1], Sy, vminY), 
                                                self.calculateX(line3[3], Sx, vminX), self.calculateY(line3[4], Sy, vminY))


            self.message_label.config(text="Object is drawn")

        self.canvas.update()
        self.canvas.focus_set() #might need to change this.

    def calculatePatch(self, patch):
        triangles = []
        points = []

        #Create bezier matrix for patches
        bezierMatrix = np.array([[-1, 3, -3, 1],
                                 [3, -6, 3, 0],
                                 [-3, 3, 0, 0],
                                 [1, 0, 0, 0]])

        bezierMatrixTranspose = np.transpose(bezierMatrix)

        #Make geometry vector for x, y, and z from the given patch.
        Gx = np.array([[patch[0][0], patch[1][0], patch[2][0], patch[3][0]],
                       [patch[4][0], patch[5][0], patch[6][0], patch[7][0]],
                       [patch[8][0], patch[9][0], patch[10][0], patch[11][0]],
                       [patch[12][0], patch[13][0], patch[14][0], patch[15][0]]])

        Gy = np.array([[patch[0][1], patch[1][1], patch[2][1], patch[3][1]],
                       [patch[4][1], patch[5][1], patch[6][1], patch[7][1]],
                       [patch[8][1], patch[9][1], patch[10][1], patch[11][1]],
                       [patch[12][1], patch[13][1], patch[14][1], patch[15][1]]])

        Gz = np.array([[patch[0][2], patch[1][2], patch[2][2], patch[3][2]],
                       [patch[4][2], patch[5][2], patch[6][2], patch[7][2]],
                       [patch[8][2], patch[9][2], patch[10][2], patch[11][2]],
                       [patch[12][2], patch[13][2], patch[14][2], patch[15][2]]])

        steps = 1 / bezierResolution

        for v in range(bezierResolution + 1):
            vCoordinate = v * steps
            vMat = np.array([[vCoordinate**3, vCoordinate**2, vCoordinate, 1]])

            for u in range(bezierResolution + 1):
                uCoordinate = u * steps
                uMat = np.array([[uCoordinate**3],
                                 [uCoordinate**2],
                                 [uCoordinate],
                                 [1]])

                Px = vMat @ bezierMatrixTranspose @ Gx @ bezierMatrix @ uMat
                Py = vMat @ bezierMatrixTranspose @ Gy @ bezierMatrix @ uMat
                Pz = vMat @ bezierMatrixTranspose @ Gz @ bezierMatrix @ uMat

                points.append([Px[0][0], Py[0][0], Pz[0][0]])

        for i in range(bezierResolution):
            for j in range(bezierResolution):
                '''
                Example for how the loop works.

                bezierResolution lets us know our triangle dimensions

                total number of squares in trangles is always:

                bezierResolution * bezierResolution

                When creating triangles our points will always get the same type of points for our triangles 
                p0 top left
                p1 top right
                p2 bottom left
                p3 bottom right

                               I
                              --->
                               
                        p0 - - p2 - - *
                        |      |      |     
                    |   |      |      |     
                  J |   p1 - - p3 - - *
                    \/  |      |      |
                        |      |      |
                        *  - - *  - - *

                        point[0] - -  point[3] - -  point[6]
                        |               |               |     
                        |               |               |     
                        point[1] - -  point[4] - -  point[7]
                        |               |               |
                        |               |               |
                        point[2]  - - point[5]  - - point[8]
                
                we use 'i' as are our anchor point to be able to get 1 corner of the triangle
                loop through triangles using the top left corner (in this example) to be able to get both triangles for the canvas
                (Bezier resolution + 1) lets us be able to get to the next column same row to get the corner besides our anchor.

                p0 = i * (bezierResolution + 1) + j
                p1 = p0 + 1
                p2 = p0 + (bezierResolution + 1)
                p3 = p2 + 1

                in this example we would go to points 
                point[0]
                point[3]
                point[1]
                point[4]

                pattern to be able to get the rest of the points: 
                p0 = i * (bezierResolution + 1) + j 
                Reason for calculation:
                    i * (bezierResolution + 1) will get us to the right column
                    + j will get us to the right row for the next triangle we are going to append.

                p1 = p0 + 1
                Reason for calculation:
                    p1 will always be the next point after our 'anchor' in our list so we just need to add 1

 
                p2 = p0 + (bezierResolution + 1)
                Reason for calculation:
                    p2 depends on bezier resolution but we are trying to get the the next "column" of our curve so we add 1 full bezierResolution to get to the next column.

                p3 = p2 + 1
                Reason for calculation:
                    p3 will always be the next point appned in our list so we just need to add 1
                '''
                p0 = i * (bezierResolution + 1) + j
                p1 = p0 + 1
                p2 = p0 + (bezierResolution + 1)
                p3 = p2 + 1

                triangles.append([points[p0], points[p1], points[p2]])
                triangles.append([points[p1], points[p2], points[p3]])

        return triangles

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

'''
When cameras are laoded in from cameras.txt we will populate list full of cameras. 
Camera is an abstract class that defines some features that both camera have and has some abstract methods for unique implemntation for the two different cameras
'''
################### Camera ###################
class Camera(ABC):
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
        self.verticies = np.array([], dtype=float)
        self.patches = np.array([], dtype=float)

    #Applies composite matrix to the copy of the cameras points.
    def applyComposite(self):
        self.verticies = np.copy(cl_world.verticies)
        self.patches = np.copy(cl_world.patches)

        for i, point in enumerate(self.verticies):
            self.verticies[i] = self.composite.dot(point)

        for patch in self.patches:
            for i , point in enumerate(patch):
                patch[i] = self.composite.dot(point)
               

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
    @abstractmethod
    def calculateComposite(self):
        pass

    @abstractmethod
    def clipLine(self):
        pass

    @abstractmethod
    def getPointLocationCode(self):
        pass
    
'''Define two different types of Cameras, parallel and perspective that inheret from the abstract class Camera'''
#Check to make sure this is good and works as intended.
class parallelCamera(Camera):
    def __init__(self, name, type, vrp, vpn, vup, prp, viewVol, viewPort):
        super().__init__(name, type, vrp, vpn, vup, prp, viewVol, viewPort)

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

        return CM

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

class perspectiveCamera(Camera):
    def __init__(self, name, type, vrp, vpn, vup, prp, viewVol, viewPort):
        super().__init__(name, type, vrp, vpn, vup, prp, viewVol, viewPort) 

    def calculateComposite(self):
        #First 4 Steps are the same between parallel and perspective. 

        #Step 1.) Translate VRP to origin
        vrpTranslation = np.array([[1, 0, 0, np.negative(self.vrp[0])],
                                [0, 1, 0, np.negative(self.vrp[1])],
                                [0, 0, 1, np.negative(self.vrp[2])],
                                [0, 0, 0, 1]])
        vrp = vrpTranslation.dot(self.vrp)

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

        '''New steps defined here for perspective projection'''

        #Get DOP (DOP = CW - PRP)
        CW = np.array([((self.viewVol[1] + self.viewVol[0])/2) - self.prp[0], ((self.viewVol[3] + self.viewVol[2])/2) - self.prp[1], 0 - self.prp[3]])
        #print(CW[0]/self.prp[2])
        #print(CW[1]/self.prp[2])

        #Step 5.) Translate PRP to the origin, Get the tranlated vrpP for calcualtion of scale matrix
        prpTranslation = np.array([[1, 0, 0, np.negative(self.prp[0])],
                                [0, 1, 0, np.negative(self.prp[1])],
                                [0, 0, 1, np.negative(self.prp[2])],
                                [0, 0, 0, 1]])
        CM = prpTranslation.dot(CM)
        vrpP = prpTranslation.dot(vrp)

        #Step 6.) Shear DOP to align with Z, Get the sheared vrpPP for calcualtion of scale matrix
        Sh = np.array([[1, 0, CW[0]/self.prp[2], 0],
                       [0, 1, CW[1]/self.prp[2], 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1],])
        CM = Sh.dot(CM)
        vrpPP = Sh.dot(vrpP)

        #tempFlag is used to get correct formulas based on position of nmin, nmax, and vrpPPz 
        tempFlag = np.abs(vrpPP[2] + self.viewVol[5]) > np.abs(vrpPP[2] + self.viewVol[4])

        #Step 7.) Scale perspective volume to be 45 degrees on both sides. Normalize the viewvolume. Calculate the zmin for clipping.
        if(tempFlag):
            xScale = (np.abs(vrpPP[2]))/(((self.viewVol[1] - self.viewVol[0])/2) * (vrpPP[2] + self.viewVol[5]))
            yScale = (np.abs(vrpPP[2]))/(((self.viewVol[3] - self.viewVol[2])/2) * (vrpPP[2] + self.viewVol[5]))
            zScale = 1/(vrpPP[2] + self.viewVol[5])
            zmin = (vrpPP[2] + self.viewVol[4])/(vrpPP[2] + self.viewVol[5])
        else:
            xScale = (np.abs(vrpPP[2]))/(((self.viewVol[1] - self.viewVol[0])/2) * (vrpPP[2] + self.viewVol[4]))
            yScale = (np.abs(vrpPP[2]))/(((self.viewVol[3] - self.viewVol[2])/2) * (vrpPP[2] + self.viewVol[4]))
            zScale = 1/(vrpPP[2] + self.viewVol[4])
            zmin = (vrpPP[2] + self.viewVol[5])/(vrpPP[2] + self.viewVol[4])

        #print(xScale)
        #print(yScale)
        #print(zScale)
        viewScale = np.array([[xScale, 0, 0, 0],
                              [0, yScale, 0, 0],
                              [0, 0, zScale, 0],
                              [0, 0, 0, 1]])
        CM = viewScale.dot(CM)              
        self.zMin = zmin

        return CM

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
                        t = (self.zMin - z0) / (z1 - z0)
                    else:
                        break
                elif outsideCode & LEFT:
                    denom = (x1 - x0 + z1 - z0)
                    if denom != 0:
                        t = (-(z0) - x0) / denom
                    else:
                        break
                elif outsideCode & RIGHT:
                    denom = (x1 - x0 - z1 - z0)
                    if denom != 0:
                        t = (z0 - x0) / denom
                    else:
                        break
                elif outsideCode & BELOW:
                    denom = (y1 - y0 + z1 - z0)
                    if denom != 0:
                        t = (-(z0) - y0) / denom
                    else:
                        break
                elif outsideCode & ABOVE:
                    denom = (y1 - y0 - z1 - z0)
                    if denom != 0:
                        t = (z0 - y0) / denom
                    else:
                        break

                #Validate t to see if it is between 0 and 1
                if not(0 <= t <= 1): 
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

                '''
                Currently having issue with location code and the way intersections are calculated that when a new intersection is calculated with the LEFT, RIGHT, ABOVE, or BELOW
                planes they can cause infinite loops when working with floating point values. This if statement prevents the loop by breaking out of it.
                '''
                if (outsideCode == code1 and code1 & outsideCode != 0) or (outsideCode == code2 and code2 & outsideCode != 0):
                    break

        if (acceptLine == True):
            return (x0/z0, y0/z0, z0, x1/z1, y1/z1, z1)

        return False

    #Get bit code for point relative to view volume with bounds x = -z, x = z, y = -z, y = z, z = zmin, z = 1
    def getPointLocationCode(self, x, y, z):
        code = INSIDE

        #Check x point relative to clipping region
        if(x < -z): 
            code |= LEFT
        elif(x > z):
            code |= RIGHT

        #Check y point relative to clipping region
        if(y < -z):
            code |= BELOW
        elif(y > z):
            code |= ABOVE

        #Check z point relative to clipping region
        if(z < self.zMin):
            code |= BEHIND
        elif(z > 1):
            code |= FRONT      

        return code  

'''Defines widgets and fucntionality of TopLevel window for the fly camera.'''
################### Camera Functionality ###################
class cl_camera_panel:
    def __init__(self, master):
        self.master = master
        
        self.camera_frame = tk.Frame(master.root)
        self.camera_frame.pack(side=tk.TOP, fill=tk.X)

        self.defineCameras()

        #Create button that when clicked makes a window
        self.flyCamera_button = tk.Button(self.camera_frame, text="Fly Camera", command = self.flyCameraWindow)
        self.flyCamera_button.pack(side=tk.LEFT)

    #When the program is first booted up the define cameras fills an array full of cameras in cl_world
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

            '''
            Creates a new camera based on the type that has been loaded in.
            We have two different types of cameras, Perspective and Parallel
            '''
            if(cameraType == "perspective"):
                cl_world.cameras.append(perspectiveCamera(cameraName, cameraType, vrp, vpn, vup, prp, viewVolume, viewPort))
            else:
                cl_world.cameras.append(parallelCamera(cameraName, cameraType, vrp, vpn, vup, prp, viewVolume, viewPort))

        #for cameras in cl_world.cameras:
            #cameras.printCameraInfo()

    def flyCameraWindow(self):
        #Create a new window that appears when the button fly window is clicked
        self.cameraWindow = tk.Toplevel(self.master.root)
        self.cameraWindow.title('Select Camera')
        self.cameraWindow.resizable(False, False)
        self.cameraWindow.geometry('750x25')

        self.cameraFrame = tk.Frame(self.cameraWindow)
        self.cameraFrame.pack(side=tk.TOP, fill=tk.X)

        self.selectLabel = tk.Label(self.cameraFrame, text = "Select camera to fly:")
        self.selectLabel.pack(side=tk.LEFT)

        #Get array of cameras for the combobox 
        cameraNames = []
        for camera in cl_world.cameras:
            cameraNames.append(camera.name)

        #Set default camera to the first name of the loaded in camera
        self.camera_string = tk.StringVar(value = cameraNames[0])
        self.dropDown = ttk.Combobox(self.cameraFrame, textvariable = self.camera_string)
        self.dropDown['values'] = cameraNames
        self.dropDown.pack(side=tk.LEFT)

        #Bind command that is called when a new camera is selected in the combobox
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

        self.Fly_Button = tk.Button(self.cameraFrame, text="Fly", width = 10, command = self.flyCamera)
        self.Fly_Button.pack(side=tk.LEFT)

    #Gets the vrp of the camera selected and puts it into the entry "VRP_Entry"
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
            cl_world.cameras[cameraToFly].vrp[0] = prevVRP[0] + (dX * (steps + 1))
            cl_world.cameras[cameraToFly].vrp[1] = prevVRP[1] + (dY * (steps + 1))
            cl_world.cameras[cameraToFly].vrp[2] = prevVRP[2] + (dZ * (steps + 1))
            cl_world.cameras[cameraToFly].composite = cl_world.cameras[cameraToFly].calculateComposite()
            self.master.draw_objects()
            self.master.canvas.update()

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
        if((cl_world.verticies.size == 0) and (cl_world.patches.size == 0)):
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

            for patch in cl_world.patches:
                for i, point in enumerate(patch):
                    patch[i] = CM.dot(point)

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
        if((cl_world.verticies.size == 0) and (cl_world.patches.size == 0)):
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
        patchesCopy = np.copy(cl_world.patches)
        patchesOriginal = np.copy(cl_world.patches)

        for i, point in enumerate(cl_world.verticies):
                verticiesCopy[i] = CM.dot(point)

        for patch in patchesCopy:
                for i, point in enumerate(patch):  
                    patch[i] = CM.dot(point)

        #Perform this in N steps (FIX THIS)
        for step in range(steps):
            for i, point in enumerate(cl_world.verticies):
                cl_world.verticies[i][0] = ((verticiesCopy[i][0] - verticiesOriginal[i][0])/steps) + cl_world.verticies[i][0]
                cl_world.verticies[i][1] = ((verticiesCopy[i][1] - verticiesOriginal[i][1])/steps) + cl_world.verticies[i][1]
                cl_world.verticies[i][2] = ((verticiesCopy[i][2] - verticiesOriginal[i][2])/steps) + cl_world.verticies[i][2]

            #Patch index to know what patch we are working on
            for patchIndex, patch in enumerate(cl_world.patches):
                for i, point in enumerate(patch):            
                    cl_world.patches[patchIndex][i][0] = ((patchesCopy[patchIndex][i][0] - patchesOriginal[patchIndex][i][0])/steps) + cl_world.patches[patchIndex][i][0]
                    cl_world.patches[patchIndex][i][1] = ((patchesCopy[patchIndex][i][1] - patchesOriginal[patchIndex][i][1])/steps) + cl_world.patches[patchIndex][i][1]
                    cl_world.patches[patchIndex][i][2] = ((patchesCopy[patchIndex][i][2] - patchesOriginal[patchIndex][i][2])/steps) + cl_world.patches[patchIndex][i][2]
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
        self.T_Entry.insert(0, '[1,1,1]') #takes in two arguments the text positon and the text itself
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
        if((cl_world.verticies.size == 0) and (cl_world.patches.size == 0)):
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

            for patch in cl_world.patches:
                for i, point in enumerate(patch):
                    patch[i] = translationMatrix.dot(point)

            self.master.draw_objects()
    
# Run the tkinter main loop
world=cl_world()
world.root.mainloop()