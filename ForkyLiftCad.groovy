import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Transform

// code here

def bearingSize = "LM10UU"
return new ICadGenerator(){
	def bearingType=Vitamins.getConfiguration("linearBallBearing", bearingSize)
	double rodDiam =bearingType.innerDiameter
	CSG vitamin_linearBallBearing_LM10UU = Vitamins.get("linearBallBearing", bearingSize)
	double rodlen = 500
	double rodEmbedlen =10
	CSG moveDHValues(CSG incoming,DHLink dh ){
		TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
		Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
		return incoming.transformed(move)
	}
	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics arg0, int arg1) {
		// TODO Auto-generated method stub
		ArrayList<CSG> back =[]
		double height =arg0.getMaxEngineeringUnits(arg1)-arg0.getMinEngineeringUnits(arg1)
		double bracing = rodlen - height - rodEmbedlen 
		CSG rod = new Cylinder(rodDiam/2, rodlen).toCSG()
		CSG upperBearing = moveDHValues(vitamin_linearBallBearing_LM10UU
										.toZMax()
										.movez(bracing)
							,arg0.getDhLink(arg1))
		CSG lowerBearing = moveDHValues(vitamin_linearBallBearing_LM10UU.movez(rodEmbedlen),arg0.getDhLink(arg1))
		CSG MyBearing = lowerBearing
						.union(upperBearing)
		MyBearing.setManipulator(arg0.getLinkObjectManipulator(arg1))
		back.add(MyBearing)
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