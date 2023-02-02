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
import javafx.scene.paint.Color

def bearingSize = "LM10UU"
return new ICadGenerator(){
			def bearingType=Vitamins.getConfiguration("linearBallBearing", bearingSize)
			double rodDiam =bearingType.innerDiameter
			CSG vitamin_linearBallBearing_LM10UU = Vitamins.get("linearBallBearing", bearingSize)
			double rodlen = 500
			double rodEmbedlen =10
			double grid =25;
			double spacing = 5;
			double calculatedTotalWidth = spacing*grid;
			double braceInsetDistance=40
			double boardThickness=6.3
			double rodToBoardDistance =20
			double sideBraceDistacne =20
			
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
				def vitamins =[]
				double boardWidth = calculatedTotalWidth*2-(braceInsetDistance*arg1)+sideBraceDistacne*2
				CSG board = new Cube(boardThickness,boardWidth,rodlen+sideBraceDistacne).toCSG()
							.toZMin()
				CSG backBoard = board
									.toXMax()
									.movex(-rodToBoardDistance)
				for(double i=-calculatedTotalWidth;i<calculatedTotalWidth+1;i+=(calculatedTotalWidth*2)) {
					def braceInsetDistanceArg1 = braceInsetDistance*arg1
					if(i<0)
						braceInsetDistanceArg1*=-1
					CSG rod = new Cylinder(rodDiam/2, rodlen).toCSG()
								.movey(i-(braceInsetDistanceArg1))
					CSG upperBearing = moveDHValues(vitamin_linearBallBearing_LM10UU
							.toZMax()
							.movez(bracing)
							,arg0.getDhLink(arg1))
					CSG lowerBearing = moveDHValues(vitamin_linearBallBearing_LM10UU.movez(rodEmbedlen),arg0.getDhLink(arg1))
					CSG MyBearing = lowerBearing
							.union(upperBearing)
							.movey(i-(braceInsetDistanceArg1))
					MyBearing.setManipulator(arg0.getLinkObjectManipulator(arg1))
					back.add(MyBearing)
					back.add(rod)
					vitamins.addAll([MyBearing,rod])
					if(arg1==0)
						rod.setManipulator(arg0.getRootListener())
					else
						rod.setManipulator(arg0.getLinkObjectManipulator(arg1-1))
				}
				
				if(arg1==0)
					backBoard.setManipulator(arg0.getRootListener())
				else
					backBoard.setManipulator(arg0.getLinkObjectManipulator(arg1-1))
				back.add(backBoard)
				
				for(CSG c:vitamins) {
					c.setColor(Color.SILVER)
					c.setMfg({incoming->return null})
				}
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