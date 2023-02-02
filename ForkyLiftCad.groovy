import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
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
			double bearingDiam = bearingType.outerDiameter
			double bearingPlasticSurround = 5
			CSG vitamin_linearBallBearing_LM10UU = Vitamins.get("linearBallBearing", bearingSize).hull()
			double rodlen = 500
			double rodEmbedlen =10
			double grid =25;
			int numGridUnits = 5;
			double calculatedTotalWidth = numGridUnits*grid;
			double braceInsetDistance=40
			double boardThickness=6.3
			double boxClearence = 8
			double rodToBoardDistance =bearingDiam/2+bearingPlasticSurround+boxClearence
			double sideBraceDistacne =20

			double frontCutoutDistance = rodEmbedlen

			double bearingBlockX = rodToBoardDistance-boxClearence*2
			double braceHeight = 100
			double bucketTopDiam = 299.7
			double bucketBottomDiam=260.0
			double bucketHeight = 442.8
			double lipHeight=106.7
			double bracingBetweenStages = rodToBoardDistance*2+boardThickness*2+boxClearence
			CSG moveDHValues(CSG incoming,DHLink dh ){
				TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
				Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
				return incoming.transformed(move)
			}
			@Override
			public ArrayList<CSG> generateCad(DHParameterKinematics kin, int linkIndex) {
				// TODO Auto-generated method stub
				ArrayList<CSG> back =[]

				
				double bracing = braceHeight
				double height =rodlen - braceHeight -rodEmbedlen
				try {
					kin.setMaxEngineeringUnits(linkIndex,height)
				}catch(Throwable t) {
					t.printStackTrace()
				}
				def vitamins =[]
				double boardWidth = calculatedTotalWidth*2-(braceInsetDistance*linkIndex)+sideBraceDistacne*2
				double bearingBlockWidth = (calculatedTotalWidth*2)-(braceInsetDistance*linkIndex*2)+bearingDiam+(bearingPlasticSurround*2)
				double bearingBlcokBearingSection =rodToBoardDistance-boxClearence
				double connectionSection= bracingBetweenStages-boardThickness*2-boxClearence*2
				CSG bearingBlock = new Cube(bearingBlcokBearingSection*2,
					bearingBlockWidth,bracing- rodEmbedlen).toCSG()
						.toZMin()
						.movez(rodEmbedlen)
						.toXMin()
						.movex(-bearingBlcokBearingSection)
				CSG connectingBlock = new Cube(bearingBlcokBearingSection+connectionSection,
							boardWidth-frontCutoutDistance*2-boxClearence,bracing- rodEmbedlen).toCSG()
								.toZMin()
								.movez(rodEmbedlen)
								.toXMin()
								.movex(-bearingBlcokBearingSection)
				bearingBlock=bearingBlock.union(connectingBlock)
				CSG board = new Cube(boardThickness,boardWidth,rodlen).toCSG()
						.toZMin()
				CSG cutout = new Cube(boardThickness,boardWidth-frontCutoutDistance*2,rodlen-frontCutoutDistance*2).toCSG()
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
					CSG clearence = new Cylinder(rodDiam/2+1, bracing-rodEmbedlen).toCSG()
					CSG upperBearing = moveDHValues(vitamin_linearBallBearing_LM10UU
							.toZMax()
							.movez(bracing)
							,kin.getDhLink(linkIndex))
					CSG lowerBearing = moveDHValues(vitamin_linearBallBearing_LM10UU.union(clearence).movez(rodEmbedlen),kin.getDhLink(linkIndex))
					CSG MyBearing = lowerBearing
							.union(upperBearing)
							.movey(i-(braceInsetDistanceArg1))
					MyBearing.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					back.add(MyBearing)
					back.add(rod)
					vitamins.addAll([MyBearing, rod])
					if(linkIndex==0)
						rod.setManipulator(kin.getRootListener())
					else
						rod.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
				}
				bearingBlock= moveDHValues(bearingBlock,kin.getDhLink(linkIndex))
									.difference(vitamins)

				if(linkIndex!=2) {
					kin.setDH_R(linkIndex, bracingBetweenStages)
				}


				def boards = [backBoard, frontBoard]
				for(CSG c:boards) {
					if(linkIndex==0) {
						c.setManipulator(kin.getRootListener())
					}else {
						c.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
					}
					c.setColor(Color.web("#EDCAA1"))
					c.addExportFormat("svg")
				}
				back.addAll(boards)
				if(linkIndex==0) {
					bearingBlock.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					bearingBlock.setColor(Color.CRIMSON)
					back.add(bearingBlock)
				}
				if(linkIndex==1) {
					bearingBlock.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					bearingBlock.setColor(Color.MEDIUMORCHID)
					back.add(bearingBlock)
				}
				if(linkIndex==2) {

					double cleatWidth = bearingBlockWidth
					double cleatBracing=20
					double bucketHeightCentering=100

					CSG cleat = ScriptingEngine.gitScriptRun("https://github.com/TechnocopiaPlant/ForkyRobot.git", "cleat.groovy", [cleatWidth, cleatBracing])
					.movey(-cleatWidth/2)
					double cleatDepth = cleat.getTotalX()
					double cleatHeight = cleat.getTotalZ()
					double cleatPlacement = rodToBoardDistance*2+boardThickness*2+boxClearence+cleatBracing+boxClearence
					kin.setDH_R(linkIndex, cleatPlacement)
					CSG bucketRim = new Cylinder(bucketTopDiam/2,lipHeight).toCSG()
							.toZMax()
					//.movez(bucketHeight)
					CSG bucket = new Cylinder(bucketBottomDiam/2,bucketTopDiam/2,bucketHeight,(int)30).toCSG()
							.toZMax()
							.union(bucketRim)
							.toXMin()
							.movez(lipHeight+cleatHeight*2+bucketHeightCentering)
					CSG bucketCleat=cleat.rotz(180)
							.movez(cleatHeight+bucketHeightCentering)
							.difference(bucket)
					CSG liftCleat = cleat.rotx(180)
							.toZMin()
							.movez(cleatHeight+bucketHeightCentering-cleatBracing)
							.movex(-cleatDepth+cleatBracing)
							.union(bearingBlock)
					bucketCleat.setColor(Color.BLUE)
					bucketCleat.setManipulator(kin.getLinkObjectManipulator(linkIndex))

					liftCleat.setColor(Color.PINK)
					liftCleat.setManipulator(kin.getLinkObjectManipulator(linkIndex))

					bucket.setColor(Color.WHITE)
					bucket.setMfg({incoming->return null})
					bucket.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					back.add(bucket)
					back.add(bucketCleat)
					back.add(liftCleat)
				}
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