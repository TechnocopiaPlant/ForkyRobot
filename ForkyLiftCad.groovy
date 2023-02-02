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
			double rodToBoardDistance =15
			double sideBraceDistacne =20
			double boxClearence = 8
			double frontCutoutDistance = 20
			CSG moveDHValues(CSG incoming,DHLink dh ){
				TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
				Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
				return incoming.transformed(move)
			}
			@Override
			public ArrayList<CSG> generateCad(DHParameterKinematics kin, int linkIndex) {
				// TODO Auto-generated method stub
				ArrayList<CSG> back =[]
				double height =kin.getMaxEngineeringUnits(linkIndex)-kin.getMinEngineeringUnits(linkIndex)
				double bracing = rodlen - height - rodEmbedlen
				def vitamins =[]
				double boardWidth = calculatedTotalWidth*2-(braceInsetDistance*linkIndex)+sideBraceDistacne*2
				CSG board = new Cube(boardThickness,boardWidth,rodlen+sideBraceDistacne).toCSG()
							.toZMin()
				CSG cutout = new Cube(boardThickness,boardWidth-frontCutoutDistance*2,rodlen+sideBraceDistacne-frontCutoutDistance*2).toCSG()
							.toZMin()
							.movez(frontCutoutDistance)
				CSG backBoard = board
									.toXMax()
									.movex(-rodToBoardDistance)
				CSG frontBoard = board.difference(cutout)
									.toXMin()
									.movex(rodToBoardDistance)
				for(double i=-calculatedTotalWidth;i<calculatedTotalWidth+1;i+=(calculatedTotalWidth*2)) {
					def braceInsetDistanceArg1 = braceInsetDistance*linkIndex
					if(i<0)
						braceInsetDistanceArg1*=-1
					CSG rod = new Cylinder(rodDiam/2, rodlen).toCSG()
								.movey(i-(braceInsetDistanceArg1))
					CSG upperBearing = moveDHValues(vitamin_linearBallBearing_LM10UU
							.toZMax()
							.movez(bracing)
							,kin.getDhLink(linkIndex))
					CSG lowerBearing = moveDHValues(vitamin_linearBallBearing_LM10UU.movez(rodEmbedlen),kin.getDhLink(linkIndex))
					CSG MyBearing = lowerBearing
							.union(upperBearing)
							.movey(i-(braceInsetDistanceArg1))
					MyBearing.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					back.add(MyBearing)
					back.add(rod)
					vitamins.addAll([MyBearing,rod])
					if(linkIndex==0)
						rod.setManipulator(kin.getRootListener())
					else
						rod.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
				}
				if(linkIndex!=2) {
					kin.setDH_R(linkIndex, rodToBoardDistance*2+boardThickness*2+boxClearence)
				}
				
				if(linkIndex==0) {
					backBoard.setManipulator(kin.getRootListener())
					frontBoard.setManipulator(kin.getRootListener())
				}else {
					backBoard.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
					frontBoard.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
				}
				back.addAll([backBoard,frontBoard])
				
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