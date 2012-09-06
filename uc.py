#!/usr/bin/python
from PyQt4 import QtGui, QtCore, QtOpenGL
import math, socket, select, struct, time, collections

def clip(x, mn, mx):
    return min(mx, max(mn, x))


class UCManager(QtCore.QObject):
    def __init__(self, remoteHost=None, serverPort=None, name=''):
        QtCore.QObject.__init__(self)
        self.name = name
        self.strokes = collections.OrderedDict()
        self.newStrokes = []
        
        self.lock = Mutex()
        
        self.net = NetworkThread(self, remoteHost, serverPort, name)
        self.net.sigReceivedData.connect(self.receiveStrokes)
        self.net.start()
        self.id = self.net.getId(timeout=5)
        
        if self.id is None:
            raise Exception("Could not determine ID")
        
        self.canvas = UnboundedCanvas(self.id)
        self.canvas.show()
        self.canvas.resize(400,400)
        self.canvas.sigCreatedStrokes.connect(self.scheduleTransfer)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.transfer)
        #self.timer.start(10)


    def scheduleTransfer(self, strokes):
        self.newStrokes.extend(strokes)
        if not self.timer.isActive():
            self.timer.start(50)

    def transfer(self):
        ## send new strokes out to the tubes
        self.net.transfer(self.newStrokes)
        self.logStrokes(self.newStrokes)
        self.newStrokes = []
        self.timer.stop()
            
    def receiveStrokes(self, data):
        ## data has arrived from network
        self.logStrokes(data)
        self.canvas.addStrokes(data)
            
    def logStrokes(self, strokes):
        with self.lock:
            for s in strokes:
                if s[0] == 's':
                    self.strokes[(s[1], s[2])] = s
                elif s[0] == 'd':
                    del self.strokes[(s[2], s[3])]
    
    def quit(self):
        self.net.quit()
        self.net.wait()

    def getAllStrokes(self):
        with self.lock:
            return self.strokes.values()




class UnboundedCanvas(QtGui.QGraphicsView):
    
    sigCreatedStrokes = QtCore.pyqtSignal(object)
    
    def __init__(self, uid):
        QtGui.QGraphicsView.__init__(self)
        self.id = uid
        self.strokes = {}  ## all strokes in scene, keyed by (origin, id)
        #self.newStrokes = []  ## self-drawn strokes generated since last call to getNewStrokes
        self.nextId = 0  ## id of next stroke to draw
        self.setScene(QtGui.QGraphicsScene())
        self.lastPos = None
        self.lastWidth = 0
        self.lastEvent = None
        self.range = QtCore.QRectF(0, 0, 1, 1)
        self.aspectLocked = True
        self.scaleCenter = False
        
        self.keysPressed = {}
        self.noRepeatKeys = [QtCore.Qt.Key_Right, QtCore.Qt.Key_Left, QtCore.Qt.Key_Up, QtCore.Qt.Key_Down, QtCore.Qt.Key_PageUp, QtCore.Qt.Key_PageDown, QtCore.Qt.Key_Q, QtCore.Qt.Key_W, QtCore.Qt.Key_A, QtCore.Qt.Key_S, QtCore.Qt.Key_Z, QtCore.Qt.Key_X, QtCore.Qt.Key_Plus, QtCore.Qt.Key_Minus, QtCore.Qt.Key_Equal]
        self.transformKeys = [QtCore.Qt.Key_Right, QtCore.Qt.Key_Left, QtCore.Qt.Key_Up, QtCore.Qt.Key_Down]
        self.antialiasDuringTransform = False
        
        self.fullScreen = False
        #self.showFullScreen()

        self.peniSize = 1.0
        self.hue = 0
        self.sat = 0
        self.val = 255

        #self.setSceneRect(QtCore.QRectF(-1e9, -1e9, 2e9, 2e9))
        self.setFrameStyle(1)
        self.setLineWidth(0)
        self.setMidLineWidth(0)
        #self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        #self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        #self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        #self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.setTransformationAnchor(QtGui.QGraphicsView.NoAnchor)
        self.setResizeAnchor(QtGui.QGraphicsView.AnchorViewCenter)
        self.setViewportUpdateMode(QtGui.QGraphicsView.MinimalViewportUpdate)
        brush = QtGui.QBrush(QtGui.QColor(0,0,0))
        self.setBackgroundBrush(brush)
        self.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self.antialias = True
        self.setOptimizationFlags(self.DontSavePainterState)
        
        self.keyTimer = QtCore.QTimer()
        self.keyTimer.timeout.connect(self.handleKeys)
        #self.keyTimer.start(30)
        
        self.penSample = QtGui.QGraphicsEllipseItem()
        self.scene().addItem(self.penSample)
        self.penSample.hide()
        self.penFader = QtCore.QTimer()
        self.penFader.timeout.connect(self.stepPenFade)
        
        

    def useOpenGL(self, b=True):
        if b:
            v = QtOpenGL.QGLWidget()
        else:
            v = QtGui.QWidget()
            
        #v.setStyleSheet("background-color: #000000;")
        self.setViewport(v)
        
    def penWidth(self):
        return self.peniSize
        
    def penColor(self):
        return QtGui.QColor.fromHsv(self.hue, self.sat, self.val)

    #def getNewStrokes(self):
        #s = self.newStrokes
        #self.newStrokes = []
        #return s

    def wheelEvent(self, ev):
        QtGui.QGraphicsView.wheelEvent(self, ev)
        sc = 1.001 ** ev.delta()
        #self.scale(sc, sc, self.viewportTransform().inverted()[0].map(ev.pos()))
        self.scale(sc, sc, self.mapToScene(ev.pos()))
        


    def mousePressEvent(self, ev):
        QtGui.QGraphicsView.mousePressEvent(self, ev)

        self.lastMousePos = ev.pos()
        self.mousePressPos = ev.pos()
        ev.accept()
        
        #self.clickAccepted = ev.isAccepted()
        #if not self.clickAccepted:
            #self.scene().clearSelection()
        
    def mouseReleaseEvent(self, ev):
        QtGui.QGraphicsView.mouseReleaseEvent(self, ev)
        self.lastButtonReleased = ev.button()
        
    def mouseMoveEvent(self, ev):
        if self.lastMousePos is None:
            self.lastMousePos = Point(ev.pos())
        delta = Point(ev.pos() - self.lastMousePos)

        QtGui.QGraphicsView.mouseMoveEvent(self, ev)
        
            
        #if self.clickAccepted:  ## Ignore event if an item in the scene has already claimed it.
            #return
        
        if ev.buttons() == QtCore.Qt.RightButton:
            delta = Point(clip(delta[0], -50, 50), clip(-delta[1], -50, 50))
            scale = 1.01 ** delta
            self.scale(scale[0], scale[1], center=self.mapToScene(self.mousePressPos))

        elif ev.buttons() == QtCore.Qt.MidButton:  ## Allow panning by left or mid button.
            px = self.pixelSize()
            tr = -delta * px
            self.translate(tr[0], tr[1])
        
        elif ev.buttons() == QtCore.Qt.LeftButton:
            p1 = self.mapToScene(self.lastMousePos)
            p2 = self.mapToScene(ev.pos())
            self.createStroke(p1, p2, 1.0, 1.0)
        self.lastMousePos = ev.pos()
            
        
    def resizeEvent(self, ev):
        self.setRange(self.range, padding=0, disableAutoPixel=False)
        self.updateMatrix()
    

    def tabletEvent(self, event):
        
        if self.lastEvent is not None:
            (x, y, press) = self.lastEvent
            wPos = QtCore.QPointF(self.mapToGlobal(QtCore.QPoint(0,0)))
            gPos = QtCore.QPointF(x, y) - wPos
            pDiff = QtCore.QPoint(gPos.x(), gPos.y()) - event.pos()
            #print "last: %0.1f, %0.1f   current:  %d, %d" % (gPos.x(), gPos.y(), event.pos().x(), event.pos().y())
            
            if abs(pDiff.x()) <= 1 and abs(pDiff.y()) <= 1:  ## this indicates the data does not suck (bugs!).
                pos = self.viewportTransform().inverted()[0].map(gPos)
                if event.pointerType() == QtGui.QTabletEvent.Pen:
                    width = press**2
                    if width > 0 or self.lastWidth > 0:
                        self.createStroke(self.lastPos, pos, self.lastWidth, width)
                    self.lastPos = pos
                    self.lastWidth = width
                elif event.pointerType() == QtGui.QTabletEvent.Eraser:
                    px = self.pixelSize()
                    r = px.x() * 20
                    path = QtGui.QPainterPath()
                    path.moveTo(pos.x()+r, pos.y())
                    path.arcTo(pos.x()-r, pos.y()-r, r*2, r*2, 0, 360)
                    path.closeSubpath()
                    if press > 0:
                        remove = []
                        for s in self.scene().items(path):
                            remove.append(('d', self.id, s.origin, s.sid))
                            self.scene().removeItem(s)
                            del self.strokes[(s.origin, s.sid)]
                        self.sigCreatedStrokes.emit(remove)
                    #item = QtGui.QGraphicsPathItem(path)
                    #item.setPen(QtGui.QPen(QtGui.QColor(255,0,0)))
                    #self.scene().addItem(item)
                else:
                    print event.pointerType()
                    
        self.lastEvent = (event.hiResGlobalPos().x(), event.hiResGlobalPos().y(), event.pressure())


    def createStroke(self, pos1, pos2, w1, w2):
        ws = self.peniSize * self.pixelSize().x()
        w1 *= ws
        w2 *= ws
        color = self.penColor()
        self.addStroke(pos1, pos2, w1, w2, color, id=self.nextId, origin=self.id)
        #self.newStrokes.append((
            #'s',
            #self.id, self.nextId,
            #pos1.x(), pos1.y(), 
            #pos2.x(), pos2.y(), 
            #w1, w2,
            #color.red(), color.green(), color.blue()
        #))
        self.sigCreatedStrokes.emit([(
            's',
            self.id, self.nextId,
            pos1.x(), pos1.y(), 
            pos2.x(), pos2.y(), 
            w1, w2,
            color.red(), color.green(), color.blue()
        )])
        
        self.nextId += 1
        

    def addStroke(self, start, stop, width1, width2, color, id, origin):
        stroke = StrokeItem(start, stop, width1, width2, color)
        
        stroke.origin = origin
        stroke.sid = id
        
        #stroke.setCacheMode(stroke.DeviceCoordinateCache)  ## degrades performance.
        self.scene().addItem(stroke)
        self.strokes[(origin, id)] = stroke


    def addStrokes(self, strokes):
        deletes = []
        for s in strokes:
            type = s[0]
            if type == 's':
                (origin, sid, x1, y1, x2, y2, w1, w2, r, g, b) = s[1:]
                self.addStroke(
                    QtCore.QPointF(x1, y1),
                    QtCore.QPointF(x2, y2),
                    w1, w2,
                    QtGui.QColor(r, g, b),
                    sid, origin
                )
            elif type == 'd':
                (o, origin, sid) = s[1:]
                deletes.append((origin, sid))
            else:
                print "Unknown stroke type:", type
        self.deleteStrokes(deletes)
        
    def deleteStrokes(self, strokes):
        for s in strokes:
            try:
                item = self.strokes[s]
                self.scene().removeItem(item)
                del self.strokes[s]
            except KeyError:
                #print "No stroke:", s
                #print self.strokes.keys()
                pass
        
    def keyPressEvent(self, ev):
        t = ev.text()
        k = ev.key()
        
        if not self.antialiasDuringTransform and ev.key() in self.transformKeys:
            self.setRenderHint(QtGui.QPainter.Antialiasing, False)
        
        if ev.key() in self.noRepeatKeys:
            ev.accept()
            if ev.isAutoRepeat():
                return
            self.keysPressed[ev.key()] = 1
            if not self.keyTimer.isActive():
                self.keyTimer.start(30)
        
        elif t == 'n':
            self.antialias = not self.antialias
            self.setRenderHint(QtGui.QPainter.Antialiasing, self.antialias)
            
        elif k == QtCore.Qt.Key_Escape:
            self.close()
        elif t == 'f':
            if self.fullScreen:
                self.showNormal()
            else:
                self.showFullScreen()
            self.fullScreen = not self.fullScreen
        else:
            ev.ignore()

    def keyReleaseEvent(self, ev):
        if not self.antialiasDuringTransform and ev.key() in self.transformKeys:
            self.setRenderHint(QtGui.QPainter.Antialiasing, self.antialias)
            
        if ev.key() in self.noRepeatKeys:
            ev.accept()
            if ev.isAutoRepeat():
                return
            try:
                del self.keysPressed[ev.key()]
            except:
                self.keysPressed = {}
            #self.evalKeyState()
            if len(self.keysPressed) == 0:
                self.keyTimer.stop()

    def handleKeys(self):  ## update behavior based on currently pressed keys
        for k in self.keysPressed:
            if k == QtCore.Qt.Key_Right:
                self.translate(30*self.pixelSize().x(), 0)
            elif k == QtCore.Qt.Key_Left:
                self.translate(-30*self.pixelSize().x(), 0)
            elif k == QtCore.Qt.Key_Up:
                self.translate(0, -30*self.pixelSize().x())
            elif k == QtCore.Qt.Key_Down:
                self.translate(0, 30*self.pixelSize().x())
                
            ## pen size
            elif k == QtCore.Qt.Key_Plus or k == QtCore.Qt.Key_Equal:
                self.peniSize *= 1.02
                self.showPenSample()
            elif k == QtCore.Qt.Key_Minus:
                self.peniSize /= 1.02
                self.showPenSample()
                
            ## color
            elif k == QtCore.Qt.Key_Q:
                self.hue = (self.hue-1) % 360
                self.showPenSample()
            elif k == QtCore.Qt.Key_W:
                self.hue = (self.hue+1) % 360
                self.showPenSample()
            elif k == QtCore.Qt.Key_A:
                self.sat = max(0, self.sat-1)
                self.showPenSample()
            elif k == QtCore.Qt.Key_S:
                self.sat = min(255, self.sat+1)
                self.showPenSample()
            elif k == QtCore.Qt.Key_Z:
                self.val = max(0, self.val-1)
                self.showPenSample()
            elif k == QtCore.Qt.Key_X:
                self.val = min(255, self.val+1)
                self.showPenSample()
                
    def showPenSample(self):
        range = self.viewRect()
        px = self.pixelSize()
        center = range.topLeft() + 10 * px
        self.penSample.setRect(QtCore.QRectF(center, center+px*50))
        self.penSample.setPen(QtGui.QPen(self.penColor(), self.penWidth()*px.x()))
        self.penSample.show()
        self.fadeOutPenSample()
        
    def fadeOutPenSample(self):
        self.penFader.start(30)
        self.penSample.setOpacity(1.0)
        
    def stepPenFade(self):
        op = self.penSample.opacity()
        op -= 0.2*(1.0 - (op-0.001))
        self.penSample.setOpacity(op)
        if op <= 0:
            self.penFader.stop()
            

    def updateMatrix(self, propagate=True):
        self.setSceneRect(self.range)
        if self.aspectLocked:
            self.fitInView(self.range, QtCore.Qt.KeepAspectRatio)
        else:
            self.fitInView(self.range, QtCore.Qt.IgnoreAspectRatio)
            
    def viewRect(self):
        """Return the boundaries of the view in scene coordinates"""
        ## easier to just return self.range ?
        r = QtCore.QRectF(self.rect())
        return self.viewportTransform().inverted()[0].mapRect(r)

    def visibleRange(self):
        ## for backward compatibility
        return self.viewRect()

    def translate(self, dx, dy):
        self.range.adjust(dx, dy, dx, dy)
        self.updateMatrix()
    
    def scale(self, sx, sy, center=None):
        scale = [sx, sy]
        if self.aspectLocked:
            scale[0] = scale[1]
        
        if self.scaleCenter:
            center = None
        if center is None:
            center = self.range.center()
            
        w = self.range.width()  / scale[0]
        h = self.range.height() / scale[1]
        self.range = QtCore.QRectF(center.x() - (center.x()-self.range.left()) / scale[0], center.y() - (center.y()-self.range.top())  /scale[1], w, h)
        
        self.updateMatrix()

    def setRange(self, newRect=None, padding=0.05, lockAspect=None, propagate=True, disableAutoPixel=True):
        if disableAutoPixel:
            self.autoPixelRange=False
        if newRect is None:
            newRect = self.visibleRange()
            padding = 0
        
        padding = Point(padding)
        newRect = QtCore.QRectF(newRect)
        pw = newRect.width() * padding[0]
        ph = newRect.height() * padding[1]
        newRect = newRect.adjusted(-pw, -ph, pw, ph)
        scaleChanged = False
        if self.range.width() != newRect.width() or self.range.height() != newRect.height():
            scaleChanged = True
        self.range = newRect
        self.updateMatrix(propagate)

    def pixelSize(self):
        """Return vector with the length and width of one view pixel in scene coordinates"""
        p0 = QtCore.QPointF(0,0)
        p1 = QtCore.QPointF(1,1)
        tr = self.transform().inverted()[0]
        p01 = tr.map(p0)
        p11 = tr.map(p1)
        return Point(p11 - p01)



class StrokeItem(QtGui.QGraphicsPathItem):
    def __init__(self, start, stop, w1, w2, color):
        dp = stop - start
        len = (dp.x()**2 + dp.y()**2) ** 0.5
        path = QtGui.QPainterPath()
        r1 = w1 * 0.5
        r2 = w2 * 0.5
        ang = math.atan2(dp.y(), dp.x())
        
        if w1 + len < w2 or w2 + len < w1:  ## just draw a circle
            path.moveTo(len, r2)
            path.arcTo(len-r2, -r2, w2, w2, 90, 360)
        
        else:  ## draw trapezoidal line with hemicircle caps
            th = math.atan2(r2-r1, len)
            thd = th * 180. / math.pi
            
            ## draw top line from previous point to new point
            path.moveTo(-r1*math.sin(th), r1*math.cos(th))
            path.lineTo(len-r2*math.sin(th), r2*math.cos(th))
            
            ## draw new line cap
            if w2 > 0:
                path.arcTo(len-r2, -r2, w2, w2, -90+thd, 180+2*thd)
                
            ## draw bottom line back toward first point
            path.lineTo(-r1*math.sin(th), -r1*math.cos(th))
            
            ## draw cap for first point
            if w1 > 0:
                path.arcTo(-r1, -r1, w1, w1, 90+thd, 180-2*thd)
        
        wAvg = (w1+w2) * 0.5
        self.area = wAvg * len + math.pi * (wAvg*0.5)**2
        self.length = len
        self.path = path
        self.w1 = w1
        self.w2 = w2
        
        QtGui.QGraphicsPathItem.__init__(self, path)
        #self.setFlag(self.ItemIgnoresTransformations)
        self.setPen(QtGui.QPen(QtCore.Qt.NoPen))
        self.setBrush(QtGui.QBrush(color))
        self.color = color
        self.setPos(start)
        self.rotate(ang * 180. / math.pi)

    ## adding LOD overrides deems to decrease performance.
    #def paint(self, painter, opts, widget):
        #lod = (opts.levelOfDetailFromTransform(painter.worldTransform()) ** 2) * self.area
        #if lod < 0.001:
            #return
        #elif lod  < .1:
            #self.color.setAlpha(255*lod)
            #pen = QtGui.QPen(self.color, 1)
            #x,y = painter.transform().map(0,0)
            #painter.resetTransform()
            #painter.setPen(pen)
            #painter.drawPoint(x,y)
            #return
        #QtGui.QGraphicsPathItem.paint(self, painter, opts, widget)








class DataStream:  ## for buffering and re-assembling data that has been broken up by network transmission
    def __init__(self):
        self.data = []
        self.length = 0
        
    def __len__(self):
        return self.length
        
    def append(self, data):
        self.data.append(data)
        self.length += len(data)
        
    def peek(self, bytes):
        """Read bytes from data stream. Return None if not enough bytes are available."""
        if bytes > len(self):
            return None
        data = []
        i = 0
        while bytes > 0:
            data.append(self.data[i][:bytes])
            bytes -= len(data[-1])
            i += 1
        return ''.join(data)
        
    def read(self, bytes):
        """Read and remove bytes from data stream. Return None if not enough bytes are available."""
        if bytes > len(self):
            return None
        data = []
        i = 0
        while bytes > 0:
            if bytes > len(self.data[0]):
                data.append(self.data.pop(0))
            else:
                data.append(self.data[0][:bytes])
                self.data.insert(0, self.data[0][bytes:])
                self.data.pop(1)
            self.length -= len(data[-1])
            bytes -= len(data[-1])
                
        return ''.join(data)
        
        


class Socket:
    ## slightly higher-level interface for sockets.
    ##  - totally asynchronous data transfer
    ##  - handles packet re-assembly
    ##  - also acts as a wrapper for previously-constructed socket objects
    
    def __init__(self, *args, **kargs):
        if len(args) == 1 and isinstance(args[0], socket.socket):
            self.socket = args[0]
        else:
            self.socket = socket.socket(*args, **kargs)
        self.socket.setblocking(0)
        self.input = DataStream()
        self.output = []
        
    def sendPacket(self, data):
        header = '!!!' + struct.pack('l', len(data))
        self.output.append(header)
        self.output.append(data)

    def readPacket(self, recv=True):
        ## return the next packet from the stream. 
        ## if recv is True, attempt to read from socket if no packets are already 
        ## available in the buffer
        
        #print "readPacket"
        header = self.input.peek(7)
        if header is None:
            if recv:
                #print "readPacket: no header yet, try recv"
                self.recvData(7)
                return self.readPacket(recv=False)
            else:
                return None
        if header[:3] != '!!!':
            raise Exception('Network data corruption.')
        dataLen = struct.unpack('l', header[3:])[0]
        #print "readPacket: packet size:", dataLen
        
        packet = self.input.read(7+dataLen)
        if packet is None:
            if recv:
                #print "readPacket: full packet not available yet, try recv"
                self.recvData(dataLen)
                return self.readPacket(recv=False)
            else:
                return None
        
        return packet[7:]
        
    def recvData(self, maxBytes=4096):
        ## read data from socket and add to input buffer.
        ## stop as soon as no data is available OR maxBytes has been reached.
        nb = 0
        while nb < maxBytes:
            try:
                data = self.socket.recv(4096)
            except socket.error as err:  
                if err.errno == 11:  ## an error here may just indicate there is nothing to receive on the socket.
                    return
                elif err.errno == 35:  ## socket temporarily unavailable; ignore
                    return
                raise
            if len(data) == 0:  ## this means socket was closed
                raise Exception('closed')
            #print "recv: got data: '%s'" % data
            nb += len(data)
            self.input.append(data)
        return nb

    def sendData(self):  ## send as much data as can be immediately handled.
        while len(self.output) > 0:
            d = self.output.pop(0)
            try:
                nb = self.socket.send(d)
            except socket.error as err:  ## an error here may just indicate the socket is not yet ready to send.
                print "socket send error:", err
                self.output.insert(0, d)
                return
            if nb < len(d):  ## if only part of the data was sent, then we're done and the rest will be sent later.
                self.output.insert(0, d[nb:])
                return



class NetworkThread(QtCore.QThread):
    
    sigReceivedData = QtCore.pyqtSignal(object)
    
    def __init__(self, manager, host=None, serverPort=None, name=''):
        QtCore.QThread.__init__(self)
        self.host = host
        self.manager = manager
        self.name = name
        self.id = None
        self.server = None
        self.listener = None
        self.lock = Mutex()
        self.inbox = []   ## strokes received from the network
        self.outbox = []  ## new strokes to send across the network
        self.stop = False
        self.readable = []
        self.writable = []
        self.socketWrappers = {}
        self.idRequests = []
        self.nextId = 0
        self.names = {}

        if self.host is not None:
            parts = self.host.split(':')
            if len(parts) == 1:
                host = self.host
                port = 34567
            else:
                host, port = parts
                port = int(port)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.connect((host, port))
                print "Connected to server at", host, port
                print "   socket:", s
                wrap = Socket(s)
                self.server = wrap
                self.readable.append(s)
                self.writable.append(s)
                self.socketWrappers[s] = wrap
                
                self.requestID(self)  ## request an ID for ourself
                self.requestHistory()
                
            except socket.error as err:
                print "Error connecting to server:", err
                
        else:
            self.id = self.nextId
            self.nextId += 1

        if serverPort is not None:
            try:
                self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                ## make sure port can be reused immediately if we restart the program
                self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
                self.listener.bind(('', serverPort))
                self.listener.listen(5)
                print "Server listening on", serverPort
                self.readable.append(self.listener)
            except socket.error as err:
                print "Could not start server.", err


    def getId(self, timeout=5):
        start = time.time()
        while True:
            with self.lock:
                uid = self.id
            if uid is None:
                time.sleep(.100)
            else:
                return uid
            if time.time() - start > timeout:
                return None

    def run(self):
            
        while True:
            
            #print "Start loop"
            #import time 
            #time.sleep(1)
            readable, writable, excepts = select.select(self.readable, self.writable, [], 0.05)
            #print "  select finished"
            ## send previously buffered data
            for sock in writable:
                if sock != self.listener:
                    self.socketWrappers[sock].sendData()
                    
            #print "  sent data"
            
            ## check for inputs
            incoming = []
            for inp in readable:
                if inp is self.listener:
                    #print "  accept incoming connection"
                    sock, addr = inp.accept()
                    wrap = Socket(sock)
                    self.readable.append(sock)
                    self.writable.append(sock)
                    self.socketWrappers[sock] = wrap
                    #print "Accepted connection from", addr, sock
                else:
                    #print "read packets from", inp
                    while True:
                        wrapper = self.socketWrappers[inp]
                        try:
                            packet = wrapper.readPacket()
                        except Exception as err:
                            if err.message == 'closed':
                                print "Socket closed:", inp
                                self.readable.remove(inp)
                                self.writable.remove(inp)
                                del self.socketWrappers[inp]
                            else:
                                raise
                        if packet is None:
                            break
                        #print "    got packet: '%s'" % packet
                        data = self.handlePacket(packet, wrapper)
                        if data is not None:
                            incoming.append(packet)
                        #print "received: '%s' (%s)" % (str(packet), str(type(packet)))

            #print "  received packets"

            ## transfer updates to threadsafe area
            with self.lock:
                if self.stop:
                    for s in self.readable:
                        s.shutdown(socket.SHUT_RDWR)
                        s.close()
                    break
                outbox = self.outbox
                self.outbox = []
                
                
            self.inbox.extend(incoming)
            self.sigReceivedData.emit(self.inbox)
            self.inbox = []

                
            #print "  transfer with app"

            ## send updates to everyone
            self.sendStrokes(outbox)

            #print "  queue outgoing packets"
            
            if len(outbox) == 0 and len(readable) == 0:
                time.sleep(0.02)
                
        print "Network thread exit"

    def handlePacket(self, packet, socket):
        ## parse a packet, return any data that should be transferred to the application
        if packet == 'i?':  ## ID requested
            if self.server is None:
                socket.sendPacket("i:" + struct.pack('l', self.nextId))
                self.nextId += 1
            else:
                self.requestID(socket)  ## forward ID request upstream
        elif packet[:2] == 'i:':  ## ID received
            recip = self.idRequests.pop(0)
            uid = struct.unpack('l', packet[2:])[0]
            if recip == self:
                self.setId(uid)
            else:
                recip.sendPacket(packet)  ## forward ID request downstream
        elif packet[:2] == 'n:':  ## someone has declared their name. Cache the data and forward upstream.
            uid = struct.unpack('l', packet[2:6])[0]
            name = packet[6:]
            self.names[uid] = name
            if self.server is not None:
                self.server.sendPacket(packet)
        elif packet[:2] == 's:': ## received strokes; send back to manager and redistribute to other sockets
            for s in self.writable:
                w = self.socketWrappers[s]
                if w is socket:
                    continue
                w.sendPacket(packet)
            self.receiveStrokes(packet[2:])
        elif packet[:2] == 'h?':  ## send all known strokes to client
            strokes = self.manager.getAllStrokes()
            for i in range(0, len(strokes), 1000):
                self.sendStrokes(strokes[i:i+1000], [socket.socket])
        else:
            print "unknown packet header:", packet[:2]

    def transfer(self, strokes):
        with self.lock:
            self.outbox.extend(strokes)
            #incoming = self.inbox
            #self.inbox = []
            #return incoming
            
    def requestID(self, recipient):
        self.idRequests.append(recipient)
        self.server.sendPacket('i?')
    
    def requestHistory(self):
        self.server.sendPacket('h?')
    
    def setId(self, uid):
        print "My ID is", uid
        with self.lock:
            self.id = uid
        if self.name is not None:
            self.server.sendPacket('n:' + struct.pack('l', uid) + self.name)  ## inform server of our name

    def receiveStrokes(self, packet):  ## called when a packet is received with new strokes in it
        formats = {
            's': '2L6d3B',
            'd': '3L'
        }
        strokes = []
        #print "parse:"
        #print packet
        while len(packet) > 0:
            type = packet[0]
            fmt = formats[type]
            size = struct.calcsize(fmt)+1
            stroke = struct.unpack(fmt, packet[1:size])
            #print size
            #print packet[:size]
            packet = packet[size:]
            
            #if stroke[1] == self.id:
                #stroke = (stroke[0], None) + stroke[2:]
            strokes.append((type,) + stroke)
        with self.lock:
            self.inbox.extend(strokes)
    
    def sendStrokes(self, strokes, sockets=None):  ## send pending strokes out to network
        if len(strokes) < 1:
            return
        #print "Sending %d strokes to" % len(strokes), sockets
        formats = {
            's': '2L6d3B',
            'd': '3L'
        }
        #print "send %d strokes" % len(strokes)
        packet = ["s:"]
        for s in strokes:
            fmt = formats[s[0]]
            #if s[1] == None:  ## replace None with self ID
                #s = (s[0], self.id) +  s[2:]
            #print "send:", s
            packet.append(s[0] + struct.pack(fmt, *s[1:]))
            #print "send:", len(packet[-1])
            #print packet[-1], 
        packetStr = ''.join(packet)
        if sockets is None:
            sockets = self.writable
            
        for s in sockets:
            wrapper = self.socketWrappers[s]
            #print "Send %d bytes to socket"%len(packetStr), wrapper
            wrapper.sendPacket(packetStr)

    

    def quit(self):
        with self.lock:
            self.stop = True
            


class Point(QtCore.QPointF):
    """Extension of QPointF which adds a few missing methods."""
    
    def __init__(self, *args):
        if len(args) == 1:
            if isinstance(args[0], QtCore.QSizeF):
                QtCore.QPointF.__init__(self, float(args[0].width()), float(args[0].height()))
                return
            elif isinstance(args[0], float) or isinstance(args[0], int):
                QtCore.QPointF.__init__(self, float(args[0]), float(args[0]))
                return
            elif hasattr(args[0], '__getitem__'):
                QtCore.QPointF.__init__(self, float(args[0][0]), float(args[0][1]))
                return
        elif len(args) == 2:
            QtCore.QPointF.__init__(self, args[0], args[1])
            return
        QtCore.QPointF.__init__(self, *args)
        
    def __len__(self):
        return 2
        
    def __reduce__(self):
        return (Point, (self.x(), self.y()))
        
    def __getitem__(self, i):
        if i == 0:
            return self.x()
        elif i == 1:
            return self.y()
        else:
            raise IndexError("Point has no index %d" % i)
        
    def __setitem__(self, i, x):
        if i == 0:
            return self.setX(x)
        elif i == 1:
            return self.setY(x)
        else:
            raise IndexError("Point has no index %d" % i)
        
    def __radd__(self, a):
        return self._math_('__radd__', a)
    
    def __add__(self, a):
        return self._math_('__add__', a)
    
    def __rsub__(self, a):
        return self._math_('__rsub__', a)
    
    def __sub__(self, a):
        return self._math_('__sub__', a)
    
    def __rmul__(self, a):
        return self._math_('__rmul__', a)
    
    def __mul__(self, a):
        return self._math_('__mul__', a)
    
    def __rdiv__(self, a):
        return self._math_('__rdiv__', a)
    
    def __div__(self, a):
        return self._math_('__div__', a)
    
    def __rpow__(self, a):
        return self._math_('__rpow__', a)
    
    def __pow__(self, a):
        return self._math_('__pow__', a)
    
    def _math_(self, op, x):
        #print "point math:", op
        #try:
            #fn  = getattr(QtCore.QPointF, op)
            #pt = fn(self, x)
            #print fn, pt, self, x
            #return Point(pt)
        #except AttributeError:
        x = Point(x)
        return Point(getattr(self[0], op)(x[0]), getattr(self[1], op)(x[1]))
    
    def length(self):
        """Returns the vector length of this Point."""
        return (self[0]**2 + self[1]**2) ** 0.5
    
    def angle(self, a):
        """Returns the angle in degrees between this vector and the vector a."""
        n1 = self.length()
        n2 = a.length()
        if n1 == 0. or n2 == 0.:
            return None
        ## Probably this should be done with arctan2 instead..
        ang = math.acos(clip(self.dot(a) / (n1 * n2), -1.0, 1.0)) ### in radians
        c = self.cross(a)
        if c > 0:
            ang *= -1.
        return ang * 180. / math.pi
    
    def dot(self, a):
        """Returns the dot product of a and this Point."""
        a = Point(a)
        return self[0]*a[0] + self[1]*a[1]
    
    def cross(self, a):
        a = Point(a)
        return self[0]*a[1] - self[1]*a[0]
        
    def proj(self, b):
        """Return the projection of this vector onto the vector b"""
        b1 = b / b.length()
        return self.dot(b1) * b1
    
    def __repr__(self):
        return "Point(%f, %f)" % (self[0], self[1])
    
    
    def min(self):
        return min(self[0], self[1])
    
    def max(self):
        return max(self[0], self[1])
        
    def copy(self):
        return Point(self)



import traceback

class Mutex(QtCore.QMutex):
    """Extends QMutex to provide warning messages when a mutex stays locked for a long time.
    Mostly just useful for debugging purposes. Should only be used with MutexLocker, not
    QMutexLocker.
    """
    
    def __init__(self, *args):
        QtCore.QMutex.__init__(self, *args)
        self.l = QtCore.QMutex()  ## for serializing access to self.tb
        self.tb = []
        self.debug = True ## True to enable debugging functions

    def tryLock(self, timeout=None, id=None):
        if timeout is None:
            l = QtCore.QMutex.tryLock(self)
        else:
            l = QtCore.QMutex.tryLock(self, timeout)

        if self.debug and l:
            self.l.lock()
            try:
                if id is None:
                    self.tb.append(''.join(traceback.format_stack()[:-1]))
                else:
                    self.tb.append("  " + str(id))
                #print 'trylock', self, len(self.tb)
            finally:
                self.l.unlock()
        return l
        
    def lock(self, id=None):
        c = 0
        waitTime = 5000  # in ms
        while True:
            if self.tryLock(waitTime, id):
                break
            c += 1
            self.l.lock()
            try:
                print "Waiting for mutex lock (%0.1f sec). Traceback follows:" % (c*waitTime/1000.)
                traceback.print_stack()
                if len(self.tb) > 0:
                    print "Mutex is currently locked from:\n", self.tb[-1]
                else:
                    print "Mutex is currently locked from [???]"
            finally:
                self.l.unlock()
        #print 'lock', self, len(self.tb)

    def unlock(self):
        QtCore.QMutex.unlock(self)
        if self.debug:
            self.l.lock()
            try:
                #print 'unlock', self, len(self.tb)
                if len(self.tb) > 0:
                    self.tb.pop()
                else:
                    raise Exception("Attempt to unlock mutex before it has been locked")
            finally:
                self.l.unlock()

    def depth(self):
        self.l.lock()
        n = len(self.tb)
        self.l.unlock()
        return n

    def traceback(self):
        self.l.lock()
        try:
            ret = self.tb[:]
        finally:
            self.l.unlock()
        return ret

    def __exit__(self, *args):
        self.unlock()

    def __enter__(self):
        self.lock()
        return self


usage = """
Options:
    -l       listen for connections
    -p port  listen on port (default is 34567)
    -c host  connect to host
    -n name  set name to use over network
"""

if __name__ == '__main__':
    QtGui.QApplication.setGraphicsSystem('raster')
    app = QtGui.QApplication([])
    import user
    import sys
    import getopt
    
    try:
        opts, ex = getopt.getopt(sys.argv[1:], 'lp:c:n:', ['listen=', 'connect=', 'name='])
    except getopt.GetoptError:
        print usage
        sys.exit(-1)
    host = None
    port = None
    name = "nobody"
    for op, val in opts:
        if op == '-l':
            port = 34567
        elif op == '-p':
            port = int(val)
        elif op == '-c':
            host = val
        elif op == '-n':
            name = val
        else:
            print "unknown option", op
            
            
    uc = UCManager(host, port, name)

    import atexit
    atexit.register(uc.quit)

    ## Start Qt event loop unless running in interactive mode.                                                                                 
    if sys.flags.interactive != 1:                                                                                                             
        app.exec_()    
        
