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
import eu.mihosoft.vrl.v3d.Toroid
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.paint.Color

def bearingSize = "LM10UU"
def pulleyBearingSize = "695zz"
CSG pulleyBearingCad = Vitamins.get("ballBearing", pulleyBearingSize).hull()
CSG nut = Vitamins.get("lockNut", "M5")
CSG boltPulley = Vitamins.get("capScrew", "M5x25")

return new ICadGenerator(){
			def bearingType=Vitamins.getConfiguration("linearBallBearing", bearingSize)
			def pulleyBearingConfig = Vitamins.getConfiguration("ballBearing", pulleyBearingSize)
			double bearingThickness = pulleyBearingConfig.width
			double rodDiam =bearingType.innerDiameter
			double bearingDiam = bearingType.outerDiameter
			double bearingHeight = bearingType.length
			double bearingPlasticSurround = 5
			CSG vitamin_linearBallBearing_LM10UU = Vitamins.get("linearBallBearing", bearingSize).hull()
			double rodlen = 500
			double rodEmbedlen =10
			double grid =25;
			int numGridUnits = 6;
			double calculatedTotalWidth = numGridUnits*grid;
			double braceInsetDistance=60
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
			double bracingBetweenStages = 0;//rodToBoardDistance*2+boardThickness*2+boxClearence
			double pulleyRadius = 10
			double cordDiameter = 6
			double pulleyBearingSeperation = 2
			double pulleySupportThickness = 4.5
			double pulleyClearenceDistance=1
			double pulleyWidth = bearingThickness*2+pulleyBearingSeperation
			double distanceBoltToPulleyOutput = pulleyRadius+cordDiameter/2
			
			CSG moveDHValues(CSG incoming,DHLink dh ){
				TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
				Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
				return incoming.transformed(move)
			}
			public HashMap<String,ArrayList<CSG>> pulleyGen(Transform location){
				HashMap<String,ArrayList<CSG>> back= []
				def toAdd= []
				def toCut=[]
				def vitamins=[]
				back.put("add",toAdd)
				back.put("cut",toCut)
				back.put("vitamins",vitamins)
				
				CSG leftBearing = pulleyBearingCad.rotx(90)
									.movey(pulleyBearingSeperation/2)
				CSG rightBearing = pulleyBearingCad.rotx(-90)
									.movey(-pulleyBearingSeperation/2)
				CSG cutout = new Toroid(pulleyRadius, pulleyRadius+cordDiameter/2).toCSG()
				CSG pulley = new Cylinder(pulleyRadius+cordDiameter/2, pulleyWidth).toCSG()
								.rotx(90)
								.movey(-pulleyWidth/2)
								.difference([leftBearing,rightBearing,cutout])
				
				CSG pulleyClearence = new Cylinder(pulleyRadius+cordDiameter+pulleyClearenceDistance, pulleyWidth+pulleyClearenceDistance).toCSG()
								.rotx(90)
								.movey(-pulleyWidth/2-pulleyClearenceDistance/2)
				
				double boltDistance = pulleyWidth/2+pulleyClearenceDistance/2+pulleySupportThickness
				
				CSG bolt = boltPulley
							.rotx(90)
							.movey(boltDistance)
				CSG nutForPulley = nut.rotx(-90)
								.movey(-boltDistance)
				CSG supportPlate = new Cylinder(pulleyRadius+cordDiameter+pulleyClearenceDistance, pulleySupportThickness).toCSG()
									.rotx(90)
									.difference(bolt)
				CSG washerCone = new Cylinder(5.5/2,4, pulleyClearenceDistance/2,30).toCSG()
									.rotx(-90)
									.difference(bolt)
				CSG leftSupport = supportPlate.toYMin()				
									.union(washerCone.rotz(180).toYMin().movey(-pulleyClearenceDistance/2))
									.movey(pulleyWidth/2+pulleyClearenceDistance/2)
				CSG rightSupport = supportPlate.toYMax()
									.union(washerCone.toYMax().movey(pulleyClearenceDistance/2))
									.movey(-pulleyWidth/2-pulleyClearenceDistance/2)
				toAdd.add(rightSupport)
				toAdd.add(leftSupport)
				vitamins.add(pulley)
				vitamins.add(leftBearing)
				vitamins.add(rightBearing)
				//vitamins.add(cutout)
				vitamins.add(bolt)
				vitamins.add(nutForPulley)
				toCut.add(pulleyClearence)
				for(def key:back.keySet()) {
					def parts = back.get(key)
					for(int i=0;i<parts.size();i++) {
						parts.set(i,parts.get(i).movex(-distanceBoltToPulleyOutput).transformed(location))
					}
				}
				return back
			}
			@Override
			public ArrayList<CSG> generateCad(DHParameterKinematics kin, int linkIndex) {
				println bearingType
				// TODO Auto-generated method stub
				ArrayList<CSG> back =[]
				int stepIndex = kin.getNumberOfLinks()-linkIndex-1

				double bracing = braceHeight
				double height =rodlen - braceHeight -rodEmbedlen
				double depthOcCSection = (2.0-linkIndex)*rodToBoardDistance*2
				try {
					kin.setMaxEngineeringUnits(linkIndex,height)
				}catch(Throwable t) {
					t.printStackTrace()
				}
				def vitamins =[]
				double stageInset = (braceInsetDistance*linkIndex)
				double lastStageInset = (braceInsetDistance*(linkIndex-1))
				double boardWidth = calculatedTotalWidth*2-stageInset*2+rodEmbedlen*2

				double bearingBlockWidth = (calculatedTotalWidth*2)-(braceInsetDistance*linkIndex*2)+bearingDiam+(bearingPlasticSurround*2)
				double bearingBlcokBearingSection =rodToBoardDistance-boxClearence
				if(linkIndex!=2) {
					kin.setDH_R(linkIndex, bracingBetweenStages)
				}else {
					bracingBetweenStages = kin.getDH_R(linkIndex)
				}
				double connectionSection= bracingBetweenStages-boardThickness*2-boxClearence*2
				println "Board distance  "+linkIndex+" is "+boardWidth
				double connectingBlockWidth = boardWidth-rodEmbedlen*2-boxClearence
				CSG bearingBlock
				CSG bearingInCShape
				if(linkIndex==2) {
					bearingBlock = new Cube(bearingBlcokBearingSection*2,
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
				}
				if(linkIndex!=0) {
					CSG supportBlock = new Cube(
							bearingBlcokBearingSection*2,
							bearingBlcokBearingSection*2,
							bracing- rodEmbedlen
							).toCSG()
							.toXMin()
							.movex(-bearingBlcokBearingSection)
							.toZMin()
							.movez(rodEmbedlen)
					bearingInCShape = supportBlock.movey(calculatedTotalWidth-lastStageInset)
							.union(supportBlock.movey(-calculatedTotalWidth+lastStageInset))
				}
				CSG board = new Cube(boardThickness,boardWidth+sideBraceDistacne*2,rodlen+sideBraceDistacne*2).toCSG()
						.toZMin()
						.movez(-sideBraceDistacne)
				CSG cutout = new Cube(boardThickness,boardWidth-rodEmbedlen*2,rodlen-rodEmbedlen*2).toCSG()
						.toZMin()
						.movez(rodEmbedlen)
				CSG backBoard = board
						.toXMax()
						.movex(-rodToBoardDistance-depthOcCSection)
				CSG frontBoard = board.difference(cutout)
						.toXMin()
						.movex(rodToBoardDistance)
				int stepOffset = 0;
				def clearenceParts=[]
				
				for(double i=-calculatedTotalWidth;i<calculatedTotalWidth+1;i+=(calculatedTotalWidth*2)) {
					def braceInsetDistanceArg1 = stageInset
					def lastBraceDist=lastStageInset
					if(i<0) {
						braceInsetDistanceArg1*=-1
						lastBraceDist*=-1
					}
					double rodSeperation = i-(braceInsetDistanceArg1)
					double lastrodSeperation = i-(lastBraceDist)

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
					
					if (linkIndex>0) {
						def dh = kin.getDhLink(linkIndex-1)
						CSG lastupperBearing = moveDHValues(vitamin_linearBallBearing_LM10UU
								.toZMax()
								.movez(bracing)
								,dh)
								.movey(lastrodSeperation)
						CSG lastlowerBearing = moveDHValues(vitamin_linearBallBearing_LM10UU.movez(rodEmbedlen),dh)
								.movey(lastrodSeperation).hull()
						clearenceParts.add(moveDHValues(clearence.movez(rodEmbedlen),dh)
								.movey(lastrodSeperation))
						clearenceParts.add(lastupperBearing)
						clearenceParts.add(lastlowerBearing)
					}
					clearenceParts.add(rod)
					upperBearing.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					lowerBearing.setManipulator(kin.getLinkObjectManipulator(linkIndex))
					def vits=[
						upperBearing,
						rod,
						lowerBearing
					]
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

				double shaftHolderY=calculatedTotalWidth*2-stageInset*2+sideBraceDistacne*2+rodEmbedlen*2
				double shaftHolderX=rodToBoardDistance*2+depthOcCSection
				CSG topBottomBlock = new Cube(shaftHolderX,shaftHolderY,sideBraceDistacne+rodEmbedlen).toCSG()
						.toXMax()
						.movex(rodToBoardDistance)
						.toZMin()
				if(linkIndex!=2) {
					CSG topBottomBlockCutout = new Cube(
							depthOcCSection + boardThickness+boxClearence,
							shaftHolderY - braceInsetDistance*2+boxClearence,
							sideBraceDistacne+rodEmbedlen).toCSG()
							.toXMax()
							.movex(rodToBoardDistance)
							.toZMin()
					topBottomBlock=topBottomBlock.difference(topBottomBlockCutout)

				}

				CSG topBlock=topBottomBlock
						.movez(rodlen+rodEmbedlen/2-sideBraceDistacne/2)
						.difference(clearenceParts)
				CSG bottomBlock = topBottomBlock
						.movez(-sideBraceDistacne)
						.difference(clearenceParts)
				if(linkIndex!=0) {
					bearingInCShape=bearingInCShape.difference(clearenceParts)
					bottomBlock=bottomBlock.union(bearingInCShape)
				}

				if(linkIndex==2) {
					bearingBlock= moveDHValues(bearingBlock,kin.getDhLink(linkIndex))
							.difference(vitamins)
							.difference(clearenceParts)
				}else {

				}



				if(linkIndex==0) {
					Transform topLeft = new Transform()
										.rotZ(90)
										.movez(rodlen+rodEmbedlen/2-sideBraceDistacne/2 + sideBraceDistacne+rodEmbedlen+4)
										.movey(calculatedTotalWidth-stageInset - cordDiameter-bearingDiam/2)
					HashMap<String,ArrayList<CSG>> bb =pulleyGen(topLeft)
					topBlock=topBlock.difference(bb.get("cut"))
					CSG pulley = bb.get("vitamins").remove(0)
					back.add(pulley)
					topBlock=topBlock.union(bb.get("add"))
					vitamins.addAll(bb.get("vitamins"))
					back.addAll(bb.get("vitamins"))
					//makeLink0( back,   connectingBlockWidth, bearingBlcokBearingSection,bearingBlock, kin,  linkIndex);
				}
				if(linkIndex==1) {
					//makeLink1( back,   connectingBlockWidth, bearingBlcokBearingSection,bearingBlock, kin,  linkIndex);
				}
				if(linkIndex==2) {

					makeLink2( back,   connectingBlockWidth, bearingBlcokBearingSection,bearingBlock, kin,  linkIndex);
				}
				for(CSG c:vitamins) {
					c.setColor(Color.SILVER)
					c.setMfg({incoming->return null})
				}
				
				
				def braceBlocks = [topBlock, bottomBlock]
				for(CSG c:braceBlocks) {
					if(linkIndex==0) {
						c.setManipulator(kin.getRootListener())
					}else {
						c.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
					}
					if(linkIndex==0)
						c.setColor(Color.web("#4FC3C8"))
					if(linkIndex==1)
						c.setColor(Color.web("#00ff00"))
					if(linkIndex==2)
						c.setColor(Color.web("#ff00ff"))
					back.add(c)
				}
				def boards = [backBoard]
				if(linkIndex==2) {
					boards.add(frontBoard)
				}
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
				topBlock.addAssemblyStep( 8+stepOffset, new Transform().movez(bearingHeight+5))
				bottomBlock.addAssemblyStep( 8+stepOffset, new Transform().movex(rodToBoardDistance*(3+stepIndex*2)))
				bottomBlock.addAssemblyStep( 9+stepOffset, new Transform().movez(-bearingHeight*(3*stepIndex +1)-5))
//				for(CSG c:clearenceParts) {
//					if(linkIndex==0) {
//						c.setManipulator(kin.getRootListener())
//					}else {
//						c.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
//					}
//					c.setColor(Color.WHITE)
//					c.setMfg({incoming->return null})
//					back.add(c)
//				}
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
				HashMap<String,ArrayList<CSG>> bb =pulleyGen(new Transform())
				def back =[]
				back.addAll(bb.get("add"))
				//back.addAll(bb.get("cut"))
				back.addAll(bb.get("vitamins"))
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