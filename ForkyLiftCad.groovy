import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder

// code here

def bearingSize = "LM10UU"
return new ICadGenerator(){
	def bearingType=Vitamins.getConfiguration("linearBallBearing", bearingSize)
	double rodDiam =bearingType.innerDiameter
	CSG vitamin_linearBallBearing_LM10UU = Vitamins.get("linearBallBearing", bearingSize)

	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics arg0, int arg1) {
		// TODO Auto-generated method stub
		ArrayList<CSG> back =[]
		double height = 180
		height=arg0.getMaxEngineeringUnits(arg1)-arg0.getMinEngineeringUnits(arg1)
		CSG rod = new Cylinder(rodDiam/2, height).toCSG()
		back.add(rod)
		if(arg1==0)
			rod.setManipulator(arg0.getRootListener())
		else
			rod.setManipulator(arg0.getLinkObjectManipulator(arg1-1))
		return back;
	}

	@Override
	public ArrayList<CSG> generateBody(MobileBase arg0) {
		// TODO Auto-generated method stub
		ArrayList<CSG> back =[]
		back.add(new Cube(1).toCSG())
		for(CSG c:back)
			c.setManipulator(arg0.getRootListener())
		for(DHParameterKinematics kin:arg0.getAllDHChains()) {
			CSG limbRoot =new Cube(1).toCSG()
			limbRoot.setManipulator(kin.getRootListener())
			back.add(limbRoot)
		}
		return back;
	}
	
	
}