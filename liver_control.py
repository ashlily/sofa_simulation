import sys
import Sofa
import socket
import math

class TutorialForceFieldLiverFEM (Sofa.PythonScriptController):
    count=0    
    '''
    HOST = ''                 # Symbolic name meaning the local host
    PORT = 50007              # Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    conn = 0;
    data = 0;
    '''
    mesh_size = 0
    visual_size = 0
    tp_index = [92,96,100,105,110]
    fp_index = [109,104,101,94,93,89,88,115,111,106,99,97,91,90,98]
    mp_index = [92,96,100,105,110]
	
    def __init__(self, node, commandLineArguments) : 
        self.count = 0
        self.sign = -1
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)

        return

    def createCube(self, parentNode, name, x, y, z, color):
	size = 0.1

        # rootNode/Spoon3
        node = parentNode.createChild(name)
        object = node.createObject('MechanicalObject',name='MecaObject',template='Rigid',scale3d=str(size)+' '+str(size)+' '+str(size))
        object.findData('position').value=str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'

        # rootNode/Spoon3/Visu
        VisuNode = node.createChild('Visu')
        VisuNode.createObject('OglModel',name='Visual',fileMesh='mesh/PokeCube.obj',color=color,scale3d=str(size)+' '+str(size)+' '+str(size))
        VisuNode.createObject('RigidMapping',input='@..',template='Rigid3d,ExtVec3f',output='@Visual') 
        
        return node

    def createGraph(self,rootNode):
	gravity = '0 0 0'

        # rootNode
        rootNode.createObject('EulerImplicitSolver', rayleighStiffness='0.1', name='Implicit Euler Solver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1e-5', tolerance='1e-5', name='Conjugate Gradient', iterations='25', template='GraphScattered')
        rootNode.createObject('VisualStyle', displayFlags='showBehavior')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('DiscreteIntersection')

        # rootNode/LiverFEM
        LiverFEM = rootNode.createChild('LiverFEM')
        self.LiverFEM = LiverFEM
        LiverFEM.gravity = gravity
        LiverFEM.createObject('MeshTopology', name='mesh', fileTopology='mesh/liver.msh')
        LiverFEM.createObject('MechanicalObject', name='dofs', template='Vec3d')
        LiverFEM.createObject('TetrahedronFEMForceField', youngModulus='50', name='FEM', poissonRatio='0.45', template='Vec3d')
        LiverFEM.createObject('UniformMass', name='mass', totalmass='1')
        LiverFEM.createObject('FixedConstraint', indices='3 39 64', name='FixedConstraint')
        #LiverFEM.createObject('PartialLinearMovementConstraint', indices='100 105 110', keyTimes='0 5 10 15 20', template='Vec3d', movements='0 0 0 -1 1 1 0.2 -0.2 -0.2 -0.5 0.5 0.5 0 0 0')
        #LiverFEM.createObject('PartialLinearMovementConstraint', indices='100 105 110', keyTimes='0 10', template='Vec3d', movements='0 0 0  0.1 -0.1 -0.1 -0.1 0.1 0.1')
        LiverFEM.createObject('PartialLinearMovementConstraint', indices='100 105 110', keyTimes='0', template='Vec3d', movements='0 0 0')

        # rootNode/LiverFEM/Visu
        Visu = LiverFEM.createChild('Visu')
        self.Visu = Visu
        Visu.gravity = gravity
        Visu.createObject('OglModel', name='VisualModel', fileMesh='mesh/liver-smooth.obj')
        Visu.createObject('BarycentricMapping', input='@../dofs', name='visual mapping', output='@VisualModel')

        # rootNode/LiverFEM/Surf
        Surf = LiverFEM.createChild('Surf')
        self.Surf = Surf
        Surf.gravity = gravity
        Surf.createObject('SphereLoader', name='SphereLoader', filename='mesh/liver.sph')
        Surf.createObject('MechanicalObject', position='@[-1].position', name='mappedMS')
        Surf.createObject('SphereModel', listRadius='@[-2].listRadius', name='CollisionModel')
        Surf.createObject('BarycentricMapping', input='@../dofs', name='sphere mapping', output='@mappedMS')
	'''
        # Spring1
        Spring1 = rootNode.createChild('Spring1')
        self.Spring1 = Spring1
        Spring1.gravity = '0 0 0'
        Spring1.createObject('MechanicalObject', position='-4 4.5 2', name='Particles', template='Vec3d', restScale='0.1')
        Spring1.createObject('UniformMass', name='Mass', template='Vec3d')
        Spring1.createObject('FixedConstraint', indices='0', name='FixedConstraint', template='Vec3d')
        rootNode.createObject('StiffSpringForceField', object1='@LiverFEM', object2='@Spring1', name='Interaction Spring', template='Vec3d', spring='110 0 1000000 0.1 0')
        
        # Spring2
        Spring2 = rootNode.createChild('Spring2')
        self.Spring2 = Spring2
        Spring2.gravity = '0 0 0'
        Spring2.createObject('MechanicalObject', position='-2 4.5 2', name='Particles', template='Vec3d', restScale='0.1')
        Spring2.createObject('UniformMass', name='Mass', template='Vec3d')
        Spring2.createObject('FixedConstraint', indices='0', name='FixedConstraint', template='Vec3d')
        rootNode.createObject('StiffSpringForceField', object1='@LiverFEM', object2='@Spring2', name='Interaction Spring', template='Vec3d', spring='92 0 1000000 0.1 0')
        '''

        # show cubes at the t points
	size = len(self.tp_index)
	for i in range(0, size):
            i_temp = self.tp_index[i]-1
	    self.createCube(self.LiverFEM, 'CubeMT' + str(i), 0, 0, 0, 'green')

        # show cubes at the f points
	size = len(self.fp_index)
	for i in range(0, size):
            i_temp = self.fp_index[i]-1
	    self.createCube(self.LiverFEM, 'CubeMF' + str(i), 0, 0, 0, 'blue')

        '''
	self.s.bind((self.HOST, self.PORT))
	self.s.listen(1)
	self.conn, self.addr = self.s.accept()
	print 'Connected by', self.addr
        '''
	return 0

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onKeyReleased(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You released a"
        return 0;

    def initGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onKeyPressed(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You pressed control+a"
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        ## usage e.g.
        #if isPressed : 
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
        return 0;

    def storeResetState(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def cleanup(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        #self.conn.close()
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print 'onGUIEvent'
        return 0;

    def onEndAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onLoaded(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def reset(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def bwdInitGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print 'onScriptEvent'
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
	
        try:
            '''
            self.data = self.conn.recv(1024)
            print "Client Says: "+self.data
            position = self.Surf.getObject('mappedMS').position[10]
            self.conn.sendall(str(self.Surf.getObject('mappedMS').position[10]))
            '''
            
            cT = self.LiverFEM.getTime()
            print 'cT = ', cT, deltaTime
            print 'position', self.LiverFEM.getObject('dofs').position[100]
            print 'position', self.LiverFEM.getObject('dofs').position[105]
            print 'position', self.LiverFEM.getObject('dofs').position[110]
            movements = self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value
            keyTimes = self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value
            print 'movements', movements#, indices='100 105 110', keyTimes='0 5 10 15 20', template='Vec3d', movements='0 0 0 -1 1 1 0.2 -0.2 -0.2 -0.5 0.5 0.5 0 0 0')
            print 'keyTimes', keyTimes#
            self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value = [[cT], [cT + 100]]
            self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [0.5*math.sin(cT), 0.5*math.cos(cT), 0]
            

            '''
            if cT > 5:
                self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value = [[0], [cT], [cT + 1]]
                self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [[0, 0, 0], [0.1, -0.1, -0.1]]
            elif cT > 10:
                self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value = [[0], [cT], [cT + 1]]
                self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [[0, 0, 0], [-0.1, 0.1, 0.1]]
            '''
            size_mp = len(self.mp_index)
	    for i in range(0, size_mp):
                i_mp = self.mp_index[i]-1

            #update scene information
	    size_fp = len(self.fp_index)
	    for i in range(0, size_fp):
                i_fp = self.fp_index[i]-1
                x = self.LiverFEM.getObject('dofs').position[i_fp][0]
	        y = self.LiverFEM.getObject('dofs').position[i_fp][1]
	        z = self.LiverFEM.getObject('dofs').position[i_fp][2]
                self.LiverFEM.getObject('CubeMF' + str(i)).findData('position').value = str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'

            size_tp = len(self.tp_index)
	    for i in range(0, size_tp):
                i_tp = self.tp_index[i]-1
                x = self.LiverFEM.getObject('dofs').position[i_tp][0]
	        y = self.LiverFEM.getObject('dofs').position[i_tp][1]
	        z = self.LiverFEM.getObject('dofs').position[i_tp][2]
                self.LiverFEM.getObject('CubeMT' + str(i)).findData('position').value = str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'

	    '''
	    if self.visual_size == 0:
	        self.visual_size = len(self.Visu.getObject('VisualModel').position)
	        for i in range(0, self.visual_size):
		    x = self.Visu.getObject('VisualModel').position[i][0]
		    y = self.Visu.getObject('VisualModel').position[i][1]
		    z = self.Visu.getObject('VisualModel').position[i][2]
		    self.createCube(self.Visu, 'CubeVM' + str(i), x, y, z, 'blue')
                #print 'visual_size = ' + str(self.visual_size)
	    else:
		#print 'visual_size = ' + str(len(self.LiverFEM.getObject('mesh').position))
	        for i in range(0, 50):
                    x = self.Visu.getObject('VisualModel').position[i][0]
	            y = self.Visu.getObject('VisualModel').position[i][1]
	            z = self.Visu.getObject('VisualModel').position[i][2]
                    #if self.LiverFEM.getObject('CubeME' + str(i)) != None: 
	            self.Visu.getObject('CubeVM' + str(i)).findData('position').value = str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'
	    '''
        except socket.error:
            print "Error Occured."
        
        return 0


def createScene(rootNode):
    rootNode.findData('dt').value = '0.02'
    rootNode.findData('gravity').value = '0 0 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    myTutorialForceFieldLiverFEM = TutorialForceFieldLiverFEM(rootNode,commandLineArguments)
    
    return 0
