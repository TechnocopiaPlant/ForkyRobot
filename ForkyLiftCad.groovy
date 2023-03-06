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
import eu.mihosoft.vrl.v3d.Parabola
import eu.mihosoft.vrl.v3d.Toroid
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.paint.Color

def bearingSize = "LM10UU"
def pulleyBearingSize = "695zz"
CSG pulleyBearingCad = Vitamins.get("ballBearing", pulleyBearingSize).hull()
CSG nut = Vitamins.get("lockNut", "M5")
CSG boltPulley = Vitamins.get("capScrew", "M5x25")
CSG vitamin_heatedThreadedInsert_M5 = Vitamins.get("heatedThreadedInsert", "M5")

return new ICadGenerator(){
			double maxPrinterDimention = 195
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
			double boardThickness=6.3
			double boxClearence = 10
			double rodToBoardDistance =bearingDiam/2+bearingPlasticSurround+boxClearence
			double frontCutoutDistance = rodEmbedlen
			double bearingBlockX = rodToBoardDistance-boxClearence*2
			double braceHeight = 100
			double bucketTopDiam = 299.7
			double bucketBottomDiam=260.0
			double bucketHeight = 442.8
			double lipHeight=106.7
			double bracingBetweenStages = 0;//rodToBoardDistance*2+boardThickness*2+boxClearence
			double pulleyRadius = bearingPlasticSurround*2+bearingDiam/2
			double cordDiameter = 6
			double pulleyBearingSeperation = 2
			double pulleySupportThickness = 4.5
			double pulleyClearenceDistance=1
			double pulleyWidth = bearingThickness*2+pulleyBearingSeperation
			double distanceBoltToPulleyOutput = pulleyRadius+cordDiameter/2
			double supportPulleyRad=pulleyRadius+cordDiameter+pulleyClearenceDistance
			double TotalPulleyCordToCord= distanceBoltToPulleyOutput*2
			double xyOfPulleyDistance = TotalPulleyCordToCord/Math.sqrt(2)

			double linkOneWidth = TotalPulleyCordToCord+4*xyOfPulleyDistance
			double linkTwoWidth = linkOneWidth+4*xyOfPulleyDistance
			double lineThreeWidth = linkTwoWidth+4*xyOfPulleyDistance
			double calculatedTotalWidth = lineThreeWidth/2;

			double braceInsetDistance=2*xyOfPulleyDistance
			double sideBraceDistacne =braceInsetDistance/2
			double pulleyClearenceDiameter=pulleyRadius+cordDiameter+pulleyClearenceDistance
			CSG moveDHValues(CSG incoming,DHLink dh ){
				TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
				Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
				return incoming.transformed(move)
			}
			public HashMap<String,ArrayList<CSG>> pulleyGen(Transform location){
				return pulleyGen( location, true)
			}
			public HashMap<String,ArrayList<CSG>> pulleyGen(Transform location, boolean useStraitPlugs){
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
						.difference([
							leftBearing,
							rightBearing,
							cutout
						])

				CSG pulleyClearenceShape = new Cylinder(pulleyClearenceDiameter/2, pulleyWidth+pulleyClearenceDistance).toCSG()
						.rotx(90)
						.movey(-pulleyWidth/2-pulleyClearenceDistance/2)

				double boltDistance = pulleyWidth/2+pulleyClearenceDistance/2+pulleySupportThickness

				CSG bolt = boltPulley
						.rotx(90)
						.movey(boltDistance)
				CSG nutForPulley = nut.rotx(-90)
						.movey(-boltDistance)
				CSG supportPlate = new Cylinder(supportPulleyRad, pulleySupportThickness).toCSG()
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
				toCut.add(pulleyClearenceShape)
				double coneStartOffset=10
				CSG cordPart =  Parabola.cone(cordDiameter*6, rodlen+coneStartOffset)
						.toZMax()
						.movez(coneStartOffset)

				CSG cord = cordPart.union(cordPart.rotx(180))
				if(useStraitPlugs)
					cord = new Cylinder(cordDiameter/2+(pulleyClearenceDistance+4), rodlen*4).toCSG()
							.movez(- rodlen*2)
				toCut.add(cord.movex(distanceBoltToPulleyOutput))
				toCut.add(cord.movex(-distanceBoltToPulleyOutput))
				for(def key:back.keySet()) {
					def parts = back.get(key)
					println key+" is "+parts
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
				double depthOcCSection = (2.0-linkIndex)*rodToBoardDistance*2+(pulleyWidth+pulleyClearenceDistance)*2+boxClearence
				try {
					kin.setMaxEngineeringUnits(linkIndex,height)
				}catch(Throwable t) {
					t.printStackTrace()
				}
				def vitamins =[]
				double stageInset = (braceInsetDistance*linkIndex)
				double lastStageInset = (braceInsetDistance*(linkIndex-1))
				double boardWidth = calculatedTotalWidth*2-stageInset*2-boxClearence
				double pulleyLocation=(calculatedTotalWidth-stageInset-xyOfPulleyDistance*2)
				double pulleyLocationBottom=pulleyLocation+2*xyOfPulleyDistance//(shaftHolderY/2 -cordDiameter-pulleyClearenceDistance*2)
				double bearingBlockWidth =( pulleyLocation+xyOfPulleyDistance/2)*2
				double bearingBlcokBearingSection =rodToBoardDistance-boxClearence
				if(linkIndex!=2) {
					kin.setDH_R(linkIndex, bracingBetweenStages)
				}else {
					bracingBetweenStages = kin.getDH_R(linkIndex)
				}
				double connectionSection= bracingBetweenStages-boardThickness*2-boxClearence*2
				println "Board distance  "+linkIndex+" is "+boardWidth
				double connectingBlockWidth = boardWidth-rodEmbedlen*2-boxClearence-xyOfPulleyDistance*2
				CSG bearingBlock
				CSG bearingInCShape
				if(linkIndex==2) {
					bearingBlock = new Cube(bearingBlcokBearingSection*2,
							bearingBlockWidth+xyOfPulleyDistance*2,bracing- rodEmbedlen).toCSG()
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
				double shaftHolderY=boardWidth
				double shaftHolderX=rodToBoardDistance*2+depthOcCSection
				double topBottomZHeight = sideBraceDistacne+rodEmbedlen
				double BackBraceHeight = sideBraceDistacne+bracing
				double cutoutDepthTotal = depthOcCSection + boardThickness+boxClearence
				double backBraceXDepth = shaftHolderX-cutoutDepthTotal
				double braceBasez=sideBraceDistacne+rodEmbedlen
				double bearingLocationOffset = xyOfPulleyDistance*1
				double cutoutWidth = shaftHolderY - braceInsetDistance*2+boxClearence
				if(cutoutWidth<supportPulleyRad*2)
					cutoutWidth=supportPulleyRad*2+1
				double frontCutoutWidth = boardWidth-rodEmbedlen*2-xyOfPulleyDistance*2
				double ZFrontCutout = rodlen-rodEmbedlen*2
				double boardZTotal =rodlen+sideBraceDistacne*2
				double zDisplacement=0

				if(linkIndex!=2) {
					frontCutoutWidth = cutoutWidth
					ZFrontCutout=boardZTotal
					zDisplacement=-rodEmbedlen-sideBraceDistacne
				}
				CSG board = new Cube(boardThickness,boardWidth,boardZTotal).toCSG()
						.toZMin()
						.movez(-sideBraceDistacne)
				CSG cutout = new Cube(boardThickness,frontCutoutWidth,ZFrontCutout).toCSG()
						.toZMin()
						.movez(rodEmbedlen+zDisplacement)
				CSG backBoard = board
						.toXMax()
						.movex(-rodToBoardDistance-depthOcCSection)
				CSG frontBoard = board.difference(cutout)
						.toXMin()
						.movex(rodToBoardDistance)
				int stepOffset = 0;
				def clearenceParts=[]
				double rodSeperationTotal=calculatedTotalWidth-bearingLocationOffset
				Transform blockXAssembly =new Transform().movex(rodToBoardDistance*(3))
				Transform blocZAssembly =new Transform().movez(-bearingHeight*2-rodlen)
				Transform bottomSwing =  new Transform().movex(100).rotY(30*(linkIndex+1))
				Transform bottomSwingNext =  new Transform().movex(100).rotY(30*(linkIndex+2))
				Transform topSplay = new Transform().movez(-100.0*(linkIndex+1))
				for(double i=-rodSeperationTotal;i<rodSeperationTotal+1;i+=(rodSeperationTotal*2)) {
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

					upperBearing.addAssemblyStep( 1+stepOffset, new Transform().movez(bearingHeight+5))
					lowerBearing.addAssemblyStep( 1+stepOffset, new Transform().movez(-(bracing+5)))
					if(linkIndex!=2) {
						upperBearing.addAssemblyStep( 8+stepOffset, blockXAssembly)
						upperBearing.addAssemblyStep( 9+stepOffset, blocZAssembly)
						upperBearing.addAssemblyStep( 2, bottomSwingNext)
						

						lowerBearing.addAssemblyStep( 8+stepOffset, blockXAssembly)
						lowerBearing.addAssemblyStep( 9+stepOffset, blocZAssembly)
						lowerBearing.addAssemblyStep( 2, bottomSwingNext)
						
					}
					rod.addAssemblyStep( 8+stepOffset, blockXAssembly)
					rod.addAssemblyStep( 9+stepOffset, blocZAssembly)
					if(linkIndex==2)
						rod.addAssemblyStep( 7+stepOffset, new Transform().movez(2*rodEmbedlen))
					else
						rod.addAssemblyStep( 7+stepOffset, new Transform().movez(rodlen+rodEmbedlen))
				}


				CSG topBottomBlock = new Cube(shaftHolderX,shaftHolderY,braceBasez).toCSG()
						.toXMax()
						.movex(rodToBoardDistance)
						.toZMin()
				//if(linkIndex!=2) {

				CSG topBottomBlockCutout = new Cube(
						cutoutDepthTotal,
						cutoutWidth,
						BackBraceHeight).toCSG()
						.toXMax()
						.movex(rodToBoardDistance)
						.toZMin()
				double filletRadius = 20
				CSG filletCubePart = new Cube(filletRadius,filletRadius,shaftHolderY).toCSG()
						.toZMin()
						.toXMax()
						.toYMin()
				CSG FilletCyl = filletCubePart.difference(new Cylinder(filletRadius, shaftHolderY).toCSG())
						.rotx(90)
						.moveToCenterY()
						.movex(filletRadius+backBraceXDepth/2)
						.movez(filletRadius+braceBasez-BackBraceHeight/2)
				CSG BackBrace = new Cube(backBraceXDepth,shaftHolderY,BackBraceHeight).toCSG()
				if(linkIndex!=2) {
					BackBrace=BackBrace
							.union(FilletCyl)
				}
				BackBrace=BackBrace
						.movex(-backBraceXDepth/2-cutoutDepthTotal+rodToBoardDistance)
						.toZMin()
				CSG IntersectionShape = new Cube(shaftHolderX,shaftHolderY,sideBraceDistacne+rodEmbedlen+supportPulleyRad*2).toCSG()
						.toXMax()
						.movex(rodToBoardDistance)
						.toZMin()
						.movez(-supportPulleyRad*2)
						.movez(-sideBraceDistacne)


				CSG topBlock=topBottomBlock
						.union(BackBrace.rotx(180).movez(braceBasez))
						.movez(rodlen+rodEmbedlen/2-sideBraceDistacne/2)
						.difference(clearenceParts)
				CSG bottomBlock = topBottomBlock
						.union(BackBrace)
						.movez(-sideBraceDistacne)
						.difference(clearenceParts)


				if(linkIndex==2) {
					bearingBlock= moveDHValues(bearingBlock,kin.getDhLink(linkIndex))
							.difference(vitamins)
							.difference(clearenceParts)
				}else {

				}

				if(linkIndex!=0) {
					CSG supportBlock = new Cylinder(
							bearingBlcokBearingSection,
							bracing- rodEmbedlen
							).toCSG()
							.movez(rodEmbedlen)
					supportBlock=supportBlock
							.union(
							supportBlock
							.movex(-(depthOcCSection + boardThickness+boxClearence)*Math.sqrt(2))
							).hull()
					bearingInCShape = supportBlock.rotz(-45).movey(calculatedTotalWidth-lastStageInset-bearingLocationOffset)
							.union(supportBlock.rotz(45).movey(-calculatedTotalWidth+lastStageInset+bearingLocationOffset))
				}
				if(linkIndex!=0) {
					bearingInCShape=bearingInCShape.difference(clearenceParts)
					bottomBlock=bottomBlock.union(bearingInCShape)
				}

				Transform bottomLeft = new Transform()
						.rotZ(-45)
						.movez(-4-sideBraceDistacne)
						.movey(pulleyLocationBottom)
				Transform bottomRight = new Transform()
						.rotZ(45)
						.movez(-4-sideBraceDistacne)
						.movey(-pulleyLocationBottom)
				Transform topVitaminsMove=new Transform().movez(bearingHeight+5)
				for(Transform tf:[bottomLeft, bottomRight]) {
					if(linkIndex!=0) {
						HashMap<String,ArrayList<CSG>> bb =pulleyGen(tf)
						def vits = bb.get("vitamins")
						CSG pulley = vits.remove(0)
						def leftBearing=vits.get(0)
						def rightBearing=vits.get(1)
						def bolt=vits.get(2)
						def nutForPulley=vits.get(3)
						back.add(pulley)
						def cuts = bb.get("cut")
						bottomBlock=bottomBlock.difference(cuts)
						topBlock=topBlock.difference(cuts)
						bottomBlock=bottomBlock.union(bb.get("add").collect{it.intersect(IntersectionShape)})
						//bb.get("vitamins").addAll(cuts)
						vitamins.addAll(bb.get("vitamins"))
						back.addAll(bb.get("vitamins"))
						if(linkIndex==0) {
							pulley.setManipulator(kin.getRootListener())
						}else {
							pulley.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
						}
						pulley.addAssemblyStep( 8+stepOffset, blockXAssembly)
						pulley.addAssemblyStep( 9+stepOffset, blocZAssembly)
						for(CSG c:bb.get("vitamins")) {
							if(linkIndex==0) {
								c.setManipulator(kin.getRootListener())
							}else {
								c.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
							}
							c.addAssemblyStep( 8+stepOffset, blockXAssembly)
							c.addAssemblyStep( 9+stepOffset, blocZAssembly)
						}
						Transform pulleyBearingSubAssembly = new Transform().movez(-100)
						pulley.addAssemblyStep( 12+stepOffset, pulleyBearingSubAssembly)
						leftBearing.addAssemblyStep( 12+stepOffset, pulleyBearingSubAssembly)
						rightBearing.addAssemblyStep( 12+stepOffset, pulleyBearingSubAssembly)

						if(tf==bottomLeft) {
							Transform boltSidePull =  new Transform().movey(25).movex(-25)
							Transform nutSidePull = new Transform().movey(-25).movex(25)
							bolt.addAssemblyStep( 13+stepOffset, boltSidePull)
							nutForPulley.addAssemblyStep( 14+stepOffset, nutSidePull)
							leftBearing.addAssemblyStep( 11+stepOffset, boltSidePull)
							rightBearing.addAssemblyStep( 11+stepOffset, nutSidePull)
						}else {
							Transform boltSidePull =  new Transform().movey(25).movex(25)
							Transform nutSidePull = new Transform().movey(-25).movex(-25)
							bolt.addAssemblyStep( 13+stepOffset, boltSidePull)
							nutForPulley.addAssemblyStep( 14+stepOffset, nutSidePull)
							leftBearing.addAssemblyStep( 11+stepOffset, boltSidePull)
							rightBearing.addAssemblyStep( 11+stepOffset, nutSidePull)

						}
					}
					bottomBlock=bottomBlock
							.difference(topBottomBlockCutout.movez(-sideBraceDistacne))
							.difference(backBoard)


				}
				Transform topLeft = new Transform()
						.rotZ(45)
						.movez(rodlen+rodEmbedlen/2-sideBraceDistacne/2 + sideBraceDistacne+rodEmbedlen+4)
						.movey(pulleyLocation)
				Transform topRight = new Transform()
						.rotZ(-45)
						.movez(rodlen+rodEmbedlen/2-sideBraceDistacne/2 + sideBraceDistacne+rodEmbedlen+4)
						.movey(-pulleyLocation)

				for(Transform tf:[topLeft, topRight]) {
					HashMap<String,ArrayList<CSG>> bb =pulleyGen(tf)
					topBlock=topBlock.difference(bb.get("cut"))
					bottomBlock=bottomBlock.difference(bb.get("cut"))
					def vits = bb.get("vitamins")
					CSG pulley = vits.remove(0)
					def leftBearing=vits.get(0)
					def rightBearing=vits.get(1)
					def bolt=vits.get(2)
					def nutForPulley=vits.get(3)
					back.add(pulley)
					topBlock=topBlock.union(bb.get("add"))
					if(linkIndex!=2)
						topBlock=topBlock
								.difference(topBottomBlockCutout.movez(rodlen+rodEmbedlen/2+sideBraceDistacne/2-bracing))
								.difference(backBoard)
					vitamins.addAll(bb.get("vitamins"))
					back.addAll(bb.get("vitamins"))
					if(linkIndex==0) {
						pulley.setManipulator(kin.getRootListener())
					}else {
						pulley.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
					}
					pulley.addAssemblyStep( 8+stepOffset, topVitaminsMove)
					for(CSG c:bb.get("vitamins")) {
						if(linkIndex==0) {
							c.setManipulator(kin.getRootListener())
						}else {
							c.setManipulator(kin.getLinkObjectManipulator(linkIndex-1))
						}
						c.addAssemblyStep( 8+stepOffset, topVitaminsMove)
					}
					Transform pulleyBearingSubAssembly = new Transform().movez(100)
					pulley.addAssemblyStep( 12+stepOffset, pulleyBearingSubAssembly)
					leftBearing.addAssemblyStep( 12+stepOffset, pulleyBearingSubAssembly)
					rightBearing.addAssemblyStep( 12+stepOffset, pulleyBearingSubAssembly)

					if(tf==topRight) {
						Transform boltSidePull =  new Transform().movey(25).movex(-25)
						Transform nutSidePull = new Transform().movey(-25).movex(25)
						bolt.addAssemblyStep( 13+stepOffset, boltSidePull)
						nutForPulley.addAssemblyStep( 14+stepOffset, nutSidePull)
						leftBearing.addAssemblyStep( 11+stepOffset, boltSidePull)
						rightBearing.addAssemblyStep( 11+stepOffset, nutSidePull)
					}else {
						Transform boltSidePull =  new Transform().movey(25).movex(25)
						Transform nutSidePull = new Transform().movey(-25).movex(-25)
						bolt.addAssemblyStep( 13+stepOffset, boltSidePull)
						nutForPulley.addAssemblyStep( 14+stepOffset, nutSidePull)
						leftBearing.addAssemblyStep( 11+stepOffset, boltSidePull)
						rightBearing.addAssemblyStep( 11+stepOffset, nutSidePull)

					}
				}

				if(linkIndex==0) {

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
				def braceBlocks = []
				CSG bottomLeftBlock=null
				CSG bottomRightBlock=null
				CSG topLeftBlock=null
				CSG topRightBlock=null
				
				if(topBlock.getTotalY()>maxPrinterDimention) {
					double cutLine = topBlock.getMaxY()-bracing-braceInsetDistance
					if(cutLine<1)
						cutLine=1;
					topLeftBlock=topBlock
						.intersect(topBlock
							.getBoundingBox()
							.toYMin()
							.movey(cutLine)
							)
					braceBlocks.add(topLeftBlock)
					
					topRightBlock=topBlock
						.intersect(topBlock
							.getBoundingBox()
							.toYMax()
							.movey(-cutLine)
							)
					braceBlocks.add(topRightBlock)
				}else {
					braceBlocks.add(topBlock)
				}
				if(bottomBlock.getTotalY()>maxPrinterDimention) {
					double cutLine = bottomBlock.getMaxY()-bracing-braceInsetDistance
					if(cutLine<1)
						cutLine=1;
					bottomLeftBlock=bottomBlock
						.intersect(bottomBlock
							.getBoundingBox()
							.toYMin()
							.movey(cutLine)
							)
					braceBlocks.add(bottomLeftBlock)
					
					bottomRightBlock=bottomBlock
						.intersect(bottomBlock
							.getBoundingBox()
							.toYMax()
							.movey(-cutLine)
							)
					braceBlocks.add(bottomRightBlock)
				}else {
					braceBlocks.add(bottomBlock)
				}

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
				//if(linkIndex==2) {
				boards.add(frontBoard)
				//}
				frontBoard.addAssemblyStep( 9, new Transform().movex(braceHeight))
				backBoard.addAssemblyStep( 4, new Transform().movez(rodlen+sideBraceDistacne*2))
				frontBoard.addAssemblyStep( 4, new Transform().movez(rodlen+sideBraceDistacne*2))

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

				if(topBlock.getTotalY()<maxPrinterDimention) {
					topBlock.addAssemblyStep( 8+stepOffset, topVitaminsMove)
					topBlock.addAssemblyStep( 2, topSplay )
				}else {
					topRightBlock.addAssemblyStep( 8+stepOffset, topVitaminsMove)
					topRightBlock.addAssemblyStep( 2, topSplay )
					topLeftBlock.addAssemblyStep( 8+stepOffset, topVitaminsMove)
					topLeftBlock.addAssemblyStep( 2, topSplay )
				}
				if(bottomBlock.getTotalY()<maxPrinterDimention) {
					bottomBlock.addAssemblyStep( 8+stepOffset, blockXAssembly)
					bottomBlock.addAssemblyStep( 9+stepOffset, blocZAssembly)
					bottomBlock.addAssemblyStep( 2, bottomSwing)
				}else {
					bottomLeftBlock.addAssemblyStep( 8+stepOffset, blockXAssembly)
					bottomLeftBlock.addAssemblyStep( 9+stepOffset, blocZAssembly)
					bottomLeftBlock.addAssemblyStep( 2, bottomSwing)
					
					bottomRightBlock.addAssemblyStep( 8+stepOffset, blockXAssembly)
					bottomRightBlock.addAssemblyStep( 9+stepOffset, blocZAssembly)
					bottomRightBlock.addAssemblyStep( 2, bottomSwing)
				}

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


				TransformNR step = new TransformNR(kin.getDhLink(linkIndex).DhStep(0)).inverse()
				Transform pulleyLocation =  com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step).apply(new Transform().rotZ(-90))
						.movey(distanceBoltToPulleyOutput)

				HashMap<String,ArrayList<CSG>> bb =pulleyGen(pulleyLocation,true)
				liftCleat=liftCleat.difference(bb.get("cut"))
				def vits = bb.get("vitamins")

				CSG pulley = vits.remove(0)
				for(CSG c:vits) {
					c.setColor(Color.SILVER)
					c.setMfg({incoming->return null})
				}
				def leftBearing=vits.get(0)
				def rightBearing=vits.get(1)
				def bolt=vits.get(2)
				def nutForPulley=vits.get(3)
				back.add(pulley)
				liftCleat=liftCleat.union(bb.get("add"))
				//bb.get("vitamins").addAll(bb.get("cut"))
				back.addAll(bb.get("vitamins"))
				pulley.setManipulator(kin.getLinkObjectManipulator(linkIndex))
				for(CSG c:bb.get("vitamins")) {
					c.setManipulator(kin.getLinkObjectManipulator(linkIndex))
				}

				Transform boltSidePull =  new Transform().movex(-30)
				Transform nutSidePull = new Transform().movex(20)
				bolt.addAssemblyStep( 5, boltSidePull)
				nutForPulley.addAssemblyStep( 6, nutSidePull)
				leftBearing.addAssemblyStep( 3, boltSidePull)
				rightBearing.addAssemblyStep( 3, nutSidePull)
				Transform pulleyBearingSubAssembly = new Transform().movez(-100)
				pulley.addAssemblyStep( 4, pulleyBearingSubAssembly)
				leftBearing.addAssemblyStep( 4, pulleyBearingSubAssembly)
				rightBearing.addAssemblyStep( 4, pulleyBearingSubAssembly)

				pulley.addAssemblyStep( 2, new Transform().movex(cleatDepth+cleatBracing))
				leftBearing.addAssemblyStep( 2, new Transform().movex(cleatDepth+cleatBracing))
				rightBearing.addAssemblyStep( 2, new Transform().movex(cleatDepth+cleatBracing))
				bolt.addAssemblyStep( 2, new Transform().movex(cleatDepth+cleatBracing))
				nutForPulley.addAssemblyStep( 2, new Transform().movex(cleatDepth+cleatBracing))

				bucketCleat.setColor(Color.BLUE)
				bucketCleat.setManipulator(kin.getLinkObjectManipulator(linkIndex))
				bucket.setColor(Color.WHITE)
				bucket.setMfg({incoming->return null})
				bucket.setManipulator(kin.getLinkObjectManipulator(linkIndex))
				liftCleat.setColor(Color.PINK)
				liftCleat.setManipulator(kin.getLinkObjectManipulator(linkIndex))
				back.add(bucket)
				back.add(bucketCleat)
				back.add(liftCleat)
				bucket.addAssemblyStep( 16, new Transform().movez(bucketHeightCentering*2))
				bucketCleat.addAssemblyStep( 16, new Transform().movez(bucketHeightCentering*2))
				bucket.addAssemblyStep(15, new Transform().movex(bucketHeightCentering*2))
				liftCleat.addAssemblyStep( 2, new Transform().movex(cleatDepth+cleatBracing))
			}
			@Override
			public ArrayList<CSG> generateBody(MobileBase arg0) {
				// TODO Auto-generated method stub
				HashMap<String,ArrayList<CSG>> bb =pulleyGen(new Transform())
				def back =[]
				//back.addAll(bb.get("add"))
				//back.addAll(bb.get("cut"))
				//back.addAll(bb.get("vitamins"))
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