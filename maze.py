from random import random
import Tkinter
import math
import lib601.util as util

class GraphicsMaze:
	def __init__(self,height,width):
		self.height = height
		self.width = width
		self.window = Tkinter.Toplevel()
		self.window.title('Dynamic Map')
		self.cell_size=5
		self.canvas = Tkinter.Canvas(self.window, width=(self.cell_size+1)*width, height=(self.cell_size+1)*height)
		self.canvas.pack()
		self.drawnCells = {}
		self.by_color = {}
		self.showSet = False
		self.showClear = False
		self.setVar = Tkinter.IntVar()
		self.clearVar = Tkinter.IntVar()
		self.setButton = Tkinter.Checkbutton(self.window, text='Show sonarHit', variable=self.setVar, command=self.doSetClear)
		self.clearButton = Tkinter.Checkbutton(self.window, text='Show sonarPass', variable=self.clearVar, command=self.doSetClear)
		self.setButton.pack()
		self.clearButton.pack()
		self.dirty = {}

	def doSetClear(self):
		self.showSet = bool(self.setVar.get())
		self.showClear = bool(self.clearVar.get())

	def update(self):
		for (loc,color) in self.dirty.iteritems():
			self.blitCell(loc,color)
		self.dirty = {}

	def markCell(self,cell,color):
		self.dirty[cell] = color

	def markCells(self,cells,color):
		for cell in cells:
			self.markCell(cell,color)

	def blitCell(self,loc,color):
		to_delete = None
		if loc in self.drawnCells:
			to_delete = self.drawnCells[loc]
		x0 = loc[1]*(self.cell_size+1)-1
		x1 = x0+self.cell_size
		y0 = loc[0]*(self.cell_size+1)-1
		y1 = y0 + self.cell_size
		self.drawnCells[loc] = self.canvas.create_rectangle(x0,y0,x1,y1,fill=color,outline=color)
		self.by_color[color] = self.by_color.get(color,set())
		self.by_color[color].add(loc)
		if to_delete is not None:
			self.canvas.delete(to_delete)

	def clearColor(self,color):
		for loc in self.by_color.get(color,[]):
			if self.isClear(loc):
				self.markCell(loc,'white')
			else:
				self.markCell(loc,'black')
		self.by_color[color] = set()

	def redrawWorld(self):
		for r in xrange(self.height):
			for c in xrange(self.width):
				if self.isClear((r,c)): #abstract method, provided by subclasses
					self.markCell((r,c),'white')
				else:
					self.markCell((r,c),'black')
					
class DynamicRobotMaze(GraphicsMaze):
	def __init__(self, height, width, x0, y0, x1, y1):
		GraphicsMaze.__init__(self, height, width) #do not remove
		self.x0,self.x1 = x0,x1
		self.y0,self.y1 = y0,y1
		self.grid = [[True for c in xrange(width)] for r in xrange(height)]

	def pointToIndices(self, point):
		ix = int(math.floor((point.x-self.x0)*self.width/(self.x1-self.x0)))
		iix = min(max(0,ix),self.width-1)
		iy = int(math.floor((point.y-self.y0)*self.height/(self.y1-self.y0)))
		iiy = min(max(0,iy),self.height-1)
		return ((self.height-1-iiy,iix))

	def indicesToPoint(self, (r,c)):
		x = self.x0 + (c+0.5)*(self.x1-self.x0)/self.width
		y = self.y0 + (self.height-r-0.5)*(self.y1-self.y0)/self.height
		return util.Point(x,y)

	def isClear(self,(r,c)):
		if not (0 <= r < self.height and 0 <= c < self.width):
			return False

		return self.grid[r][c]

	def isPassable(self,(r,c)):
		size = 4
		for y in range(-size, size+1):
			for x in range(-size, size+1):
				if not self.isClear((r+y, c+x)):
					return False

		return self.isClear((r,c))

	def sonarHit(self,(r,c)):
		self.grid[r][c] = False
		#don't remove graphics commands below
		if self.showSet:
			self.markCell((r,c),'orange')
		elif self.isClear((r,c)):
			self.markCell((r,c),'white')
		else:
			self.markCell((r,c),'black')
	
	def sonarPass(self,(r,c)):
		self.grid[r][c] = True
		#don't remove graphics commands below
		if self.showClear:
			self.markCell((r,c),'cyan')
		elif self.isClear((r,c)):
			self.markCell((r,c),'white')
		else:
			self.markCell((r,c),'black')
