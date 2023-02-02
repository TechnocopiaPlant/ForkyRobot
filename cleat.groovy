import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Wedge

double depth = 25


if (args==null)
	args=[100,10]
double inset = args[1]
double length = args[0]
CSG bracket=new Cube(depth+inset,length,depth+inset).toCSG()
				.toZMin()
				.toYMin()
				.toXMin()
				.movex(-inset)
CSG cleatWedge =bracket.difference( new Wedge(depth,length,depth).toCSG())

					
return cleatWedge