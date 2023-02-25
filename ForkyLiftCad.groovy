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
			double bearingHeight = bearingType.length
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
			double sideBraceDistacne =braceInsetDistance/2

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
				println bearingType
				// TODO Auto-generated method stub
				ArrayList<CSG> back =[]
				int stepIndex = kin.getNumberOfLinks()-linkIndex-1

				double bracing = braceHeight
				double height =rodlen - braceHeight -rodEmbedlen
				try {
					kin.setMaxEngineeringUnits(linkIndex,height)
				}catch(Throwable t) {
					t.printStackTrace()
				}
				def vitamins =[]
				double stageInset = (braceInsetDistance*linkIndex)
				double boardWidth = calculatedTotalWidth*2-stageInset*2+rodEmbedlen*2
				double bearingBlockWidth = (calculatedTotalWidth*2)-(braceInsetDistance*linkIndex*2)+bearingDiam+(bearingPlasticSurround*2)
				double bearingBlcokBearingSection =rodToBoardDistance-boxClearence
				double connectionSection= bracingBetweenStages-boardThickness*2-boxClearence*2
				println "Board distance  "+linkIndex+" is "+boardWidth
				double connectingBlockWidth = boardWidth-rodEmbedlen*2-boxClearence
				CSG bearingBlock = new Cube(bearingBlcokBearingSection*2,
						bearingBlockWidth,bracing- rodEmbedlen).toCSG()
						.toZMin()
						.movez(rodEmbedlen)
						.toXMin()
						.movex(-bearingBlcokBearingSection)
				CSG connectingBlock = new Cube(bearingBlcokBearingSection+connectionSection,
						connectingBlockWidth,bracing- rodEmbedlen).toCSG()
						.toZMin()
						.movez(rodEmbedlen)
						.toXMin()
						.movex(-bearingBlcokBearingSection)
				bearingBlock=bearingBlock.union(connectingBlock)
				CSG board = new Cube(boardThickness,boardWidth+sideBraceDistacne*2,rodlen+sideBraceDistacne*2).toCSG()
						.toZMin()
						.movez(-sideBraceDistacne)
				CSG cutout = new Cube(boardThickness,boardWidth-rodEmbedlen*2,rodlen-rodEmbedlen*2).toCSG()
						.toZMin()
						.movez(rodEmbedlen)
				CSG backBoard = board
						.toXMax()
						.movex(-rodToBoardDistance)
				CSG frontBoard = board.difference(cutout)
						.toXMin()
						.movex(rodToBoardDistance)
				int stepOffset = 0;
				def clearenceParts=[]
				for(double i=-calculatedTotalWidth;i<calculatedTotalWidth+1;i+=(calculatedTotalWidth*2)) {
					def braceInsetDistanceArg1 = stageInset
					if(i<0)
						braceInsetDistanceArg1*=-1
					double rodSeperation = i-(braceInsetDistanceArg1)
					println "Seperation dist for "+linkIndex+" is "+(rodSeperation*2)
					CSG rod = new Cylinder(rodDiam/2, rodlen).toCSG()
							.movey(rodSeperation)
					CSG clearence = new Cylinder(rodDiam/2+1, bracing-rodEmbedlen).toCSG()
					CSG upperBearing = moveDHValues(vitamin_linearBallBearing_LM10UU
							.toZMax()
							.movez(bracing)
							,kin.getDhLink(linkIndex))
							.movey(rodSeperation)
					CSG lowerBearing = moveDHValues(vitamin_linearBallBearing_LM10UU.movez(rodEmbedlen),kin.getDhLink(linkIndex))
										.movey(rodSeperation).hull()
					clearenceParts.add(moveDHValues(clearence.movez(rodEmbedlen),kin.getDhLink(linkIndex))
										.movey(rodSeperation))
							
					upperBearing.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					lowerBearing.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					def vits=[upperBearing, rod,lowerBearing]
					back.addAll(vits)
					vitamins.addAll(vits)
					if(linkIndex==0)
						rod.setManipulator(kin.getRootListener())
					else
						rod.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
					
					upperBearing.addAssemblyStep( 2+stepOffset, new Transform().movez(bearingHeight+5))
					lowerBearing.addAssemblyStep( 2+stepOffset, new Transform().movez(-(bracing+5)))
					rod.addAssemblyStep( 7+stepOffset, new Transform().movez(-rodlen-braceInsetDistance-bracing))
				}
				CSG topBottomBlock = new Cube(rodToBoardDistance*2,calculatedTotalWidth*2-stageInset*2+sideBraceDistacne*2+rodEmbedlen*2,sideBraceDistacne+rodEmbedlen).toCSG()
				
				CSG topBlock=topBottomBlock
								.movez(rodlen+rodEmbedlen/2)
								.difference(vitamins)
				CSG bottomBlock = topBottomBlock
								.movez(-rodEmbedlen/2)
								.difference(vitamins)
				topBlock.addAssemblyStep( 8+stepOffset, new Transform().movez(bearingHeight+5))
				bottomBlock.addAssemblyStep( 8+stepOffset, new Transform().movex(rodToBoardDistance*(3+stepIndex*2)))
				bottomBlock.addAssemblyStep( 9+stepOffset, new Transform().movez(-bearingHeight*(3*stepIndex +1)-5))
				
				bearingBlock= moveDHValues(bearingBlock,kin.getDhLink(linkIndex))
						.difference(vitamins)
						.difference(clearenceParts)

				if(linkIndex!=2) {
					kin.setDH_R(linkIndex, bracingBetweenStages)
				}

				def braceBlocks = [topBlock,bottomBlock]
				for(CSG c:braceBlocks) {
					if(linkIndex==0) {
						c.setManipulator(kin.getRootListener())
					}else {
						c.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
					}
					c.setColor(Color.color(Math.random(), Math.random(), Math.random()))
					back.add(c)
				}
				def boards = [backBoard, frontBoard]
				frontBoard.addAssemblyStep( 4, new Transform().movex(braceHeight))
				backBoard.addAssemblyStep( 3, new Transform().movey(boardWidth*2))
				frontBoard.addAssemblyStep( 3, new Transform().movey(boardWidth*2))
				
				for(CSG c:boards) {
					if(linkIndex==0) {
						c.setManipulator(kin.getRootListener())
					}else {
						c.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
					}
					c.setColor(Color.web("#EDCAA1"))
					c.addExportFormat("svg")
					back.add(c)
				}
				if(linkIndex==0) {
					makeLink0( back,   connectingBlockWidth, bearingBlcokBearingSection,bearingBlock, kin,  linkIndex);
				}
				if(linkIndex==1) {
					makeLink1( back,   connectingBlockWidth, bearingBlcokBearingSection,bearingBlock, kin,  linkIndex);
				}
				if(linkIndex==2) {

					makeLink2( back,   connectingBlockWidth, bearingBlcokBearingSection,bearingBlock, kin,  linkIndex);
				}
				for(CSG c:vitamins) {

					c.setColor(Color.SILVER)
					c.setMfg({incoming->return null})
				}
				return back;
			}
			private void makeLink0(ArrayList<CSG> back, double  connectingBlockWidth,double bearingBlcokBearingSection,CSG bearingBlock,DHParameterKinematics kin, int linkIndex) {
				bearingBlock.setManipulator(kin.getLinkObjectManipulator(linkIndex))
				bearingBlock.setColor(Color.CRIMSON)
				back.add(bearingBlock)
				bearingBlock.addAssemblyStep( 1, new Transform().movex(-bearingBlcokBearingSection))
			}
			private void makeLink1(ArrayList<CSG> back, double  connectingBlockWidth,double bearingBlcokBearingSection,CSG bearingBlock,DHParameterKinematics kin, int linkIndex) {
				bearingBlock.setManipulator(kin.getLinkObjectManipulator(linkIndex))
				bearingBlock.setColor(Color.MEDIUMORCHID)
				back.add(bearingBlock)
				bearingBlock.addAssemblyStep( 1, new Transform().movex(-bearingBlcokBearingSection))
			}
			
			private void makeLink2(ArrayList<CSG> back, double  connectingBlockWidth,double bearingBlcokBearingSection,CSG bearingBlock,DHParameterKinematics kin, int linkIndex) {
				double cleatWidth = connectingBlockWidth
				double cleatBracing=20
				double bucketHeightCentering=60

				CSG cleat = ((CSG)ScriptingEngine.gitScriptRun("https://github.com/TechnocopiaPlant/ForkyRobot.git", "cleat.groovy", [cleatWidth, cleatBracing]))
						.movey(-cleatWidth/2)
				double cleatDepth = cleat.getTotalX()
				double cleatHeight = cleat.getTotalZ()
				double cleatPlacement = rodToBoardDistance*2+boardThickness*2+boxClearence+cleatBracing+boxClearence
				kin.setDH_R(linkIndex, cleatPlacement)
				CSG heel = new Cube(cleatPlacement*2-bearingBlcokBearingSection*2-boardThickness*2-boxClearence*4,cleatWidth,bucketHeightCentering*2+cleatHeight).toCSG()
						.toZMin()
						.movez(-bucketHeightCentering)


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
						.union(heel)
						.difference(bucket)
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
				bucket.addAssemblyStep( 10, new Transform().movez(bucketHeightCentering*2))
				bucketCleat.addAssemblyStep( 10, new Transform().movez(bucketHeightCentering*2))
				bucket.addAssemblyStep(9, new Transform().movex(bucketHeightCentering*2))
				
				liftCleat.addAssemblyStep( 1, new Transform().movex(cleatDepth+cleatBracing))
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