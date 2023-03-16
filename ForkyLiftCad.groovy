import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.google.gson.Gson
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine

import java.lang.reflect.Type;

import org.apache.commons.io.FileUtils
import org.apache.commons.io.IOUtils

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.Parabola
import eu.mihosoft.vrl.v3d.Toroid
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.paint.Color
import javafx.scene.transform.Affine

def bearingSize = "LM10UU"
def pulleyBearingSize = "695zz"
CSG pulleyBearingCad = Vitamins.get("ballBearing", pulleyBearingSize).hull()
CSG nut = Vitamins.get("lockNut", "M5")
CSG boltPulley = Vitamins.get("capScrew", "M5x25")
CSG insert = Vitamins.get("heatedThreadedInsert", "M5")

String HTTPS_GITHUB_COM_TECHNOCOPIA_PLANT_FORKY_ROBOT_GIT = "https://github.com/TechnocopiaPlant/ForkyRobot.git"
Type TT_mapStringString = new TypeToken<HashMap<String, Double>>() {}.getType();
Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();
File	cachejson = ScriptingEngine.fileFromGit(HTTPS_GITHUB_COM_TECHNOCOPIA_PLANT_FORKY_ROBOT_GIT, "baseDim.json")
def inPut = FileUtils.openInputStream(cachejson);
def jsonString = IOUtils.toString(inPut);
HashMap<String, Double> database = gson.fromJson(jsonString, TT_mapStringString);



return new ICadGenerator(){

			Transform liftCleatAssembly  =new Transform().movex(50)
			double gridUnits = database.baseBad
			def wheelbaseIndex = database.wheelbaseIndex
			def wheelbaseIndexY = database.wheelbaseIndexY
			def rideHeight = database.rideHeight
			def plateThickness =database.plateThickness
			def plateLevel = rideHeight+plateThickness
			def wheelbase=gridUnits*wheelbaseIndex
			double electronicsBayStandoff = database.electronicsBayStandoff
			double cleatDepthVal = database.cleatDepth
			
			def bearingType=Vitamins.getConfiguration("linearBallBearing", bearingSize)
			def pulleyBearingConfig = Vitamins.getConfiguration("ballBearing", pulleyBearingSize)
			double bearingThickness = pulleyBearingConfig.width
			double rodDiam =bearingType.innerDiameter
			double bearingDiam = bearingType.outerDiameter
			double bearingHeight = bearingType.length

			CSG vitamin_linearBallBearing_LM10UU = Vitamins.get("linearBallBearing", bearingSize).hull()


			double bearingPlasticSurround = 5
			double maxPrinterDimention = 195
			double rodlen = 500
			double rodEmbedlen =10
			double boardThickness=6.3
			double boxClearence = 10
			double cleatBracing=20
			double bucketHeightCentering=60
			double braceHeight = 100
			double bucketTopDiam = 299.7
			double bucketBottomDiam=260.0
			double bucketHeight = 442.8
			double lipHeight=106.7
			double bracingBetweenStages = 0;
			double cordDiameter = 6
			double pulleyBearingSeperation = 2
			double pulleySupportThickness = 4.5
			double pulleyClearenceDistance=1
			double cleatBracingDepthDH=101.0

			double rodToBoardDistance =bearingDiam/2+bearingPlasticSurround+boxClearence
			double cleatPlacement = rodToBoardDistance*2+boardThickness*2+boxClearence+cleatBracing+boxClearence
			double frontCutoutDistance = rodEmbedlen
			double bearingBlockX = rodToBoardDistance-boxClearence*2
			double pulleyRadius = bearingPlasticSurround*2+bearingDiam/2
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
			double CLEAT_PLACEMENT_BUCKET_TOP_DIAM = -cleatPlacement+(bucketTopDiam/3)//-wheelbase/2
			double connectingBlockWidth = calculatedTotalWidth*2-(braceInsetDistance*2)*2-boxClearence-rodEmbedlen*2-boxClearence-xyOfPulleyDistance*2
			double bearingBlcokBearingSection =rodToBoardDistance-boxClearence
			double connectionSection= cleatBracingDepthDH-boardThickness*2-boxClearence*2
			double cleatHeight = cleatBracing+cleatDepthVal
			double kinematicsToBottomOfBucket = bucketHeight-(lipHeight+cleatHeight*2+bucketHeightCentering)
			double zHeightOfLiftKinematics = kinematicsToBottomOfBucket+plateLevel+electronicsBayStandoff+plateThickness
			
			TransformNR baseOfArmFromCenter = new TransformNR(wheelbase/2,
				CLEAT_PLACEMENT_BUCKET_TOP_DIAM,
				zHeightOfLiftKinematics
				, new RotationNR(0,90,0))
			Transform baseOfArmFromCentercsg =TransformFactory.nrToCSG(baseOfArmFromCenter)
			
			
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
				CSG coreCutout = new Cylinder(5.75/2, pulleyWidth).toCSG()
				CSG pulley = new Cylinder(pulleyRadius+cordDiameter/2, pulleyWidth).toCSG()
						.difference(coreCutout)
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
						def partsGetMovexTransformed = parts.get(i).movex(-distanceBoltToPulleyOutput).transformed(location)
						partsGetMovexTransformed.setMfg({incoming->
							return incoming.transformed(location.inverse()).rotx(90).toZMin()

						})
						parts.set(i,partsGetMovexTransformed)
					}
				}
				return back
			}
			@Override
			public ArrayList<CSG> generateCad(DHParameterKinematics kin, int linkIndex) {
				println bearingType
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
				if(linkIndex!=2) {
					kin.setDH_R(linkIndex, 0)
				}else
					kin.setDH_R(linkIndex, cleatBracingDepthDH)
				bracingBetweenStages = kin.getDH_R(linkIndex)
				
				println "Board distance  "+linkIndex+" is "+boardWidth
				CSG bearingInCShape
				if(linkIndex==2) {

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
				Transform bottomSwing =  new Transform().movex(100).movez(100*(linkIndex+1))
				Transform bottomSwingNext =  new Transform().movex(100).movez(100*(linkIndex+2))
				Transform topSplay = new Transform().movez(-100.0*(linkIndex+1))
				Affine frameListener = null
				if(linkIndex==0)
					frameListener=kin.getRootListener()
				else
					frameListener=kin.getLinkObjectManipulator(linkIndex-1)
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
					rod.setManipulator(frameListener)

					upperBearing.addAssemblyStep( 1+stepOffset, new Transform().movez(bearingHeight+5))
					lowerBearing.addAssemblyStep( 1+stepOffset, new Transform().movez(-(bearingHeight+5)))
					if(linkIndex!=2) {
						upperBearing.addAssemblyStep( 8+stepOffset, blockXAssembly)
						upperBearing.addAssemblyStep( 9+stepOffset, blocZAssembly)
						upperBearing.addAssemblyStep( 2, bottomSwingNext)


						lowerBearing.addAssemblyStep( 8+stepOffset, blockXAssembly)
						lowerBearing.addAssemblyStep( 9+stepOffset, blocZAssembly)
						lowerBearing.addAssemblyStep( 2, bottomSwingNext)

					}else {
						upperBearing.addAssemblyStep( 2, liftCleatAssembly)
						lowerBearing.addAssemblyStep( 2, liftCleatAssembly)
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
				def SideBoards = []
				if(linkIndex==0) {
					CSG SideBoard = new Cube(shaftHolderX,boardThickness,boardZTotal).toCSG()
									.toXMax()
									.movex(rodToBoardDistance)
									.toZMin()
									.movez(-sideBraceDistacne)
					def sideBoardToYMaxMovey = SideBoard.toYMax().movey(-shaftHolderY/2)
					SideBoards.add(sideBoardToYMaxMovey)
					def sideBoardToYMinMovey = SideBoard.toYMin().movey(shaftHolderY/2)
					SideBoards.add(sideBoardToYMinMovey)
					sideBoardToYMaxMovey.setName("sideBoardRight")
					sideBoardToYMinMovey.setName("sideBoardLeft")
				}
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
				int pulleyIndex=1
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
						pulley.setManipulator(frameListener)
						pulley.setName("Bottom-Pulley-"+pulleyIndex+"-link-"+linkIndex)

						pulley.addAssemblyStep( 8+stepOffset, blockXAssembly)
						pulley.addAssemblyStep( 9+stepOffset, blocZAssembly)
						for(CSG c:bb.get("vitamins")) {
							c.setManipulator(frameListener)

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
				pulleyIndex=1
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
					pulley.setName("Top-Pulley-"+pulleyIndex+"-link-"+linkIndex)
					pulleyIndex++
					back.add(pulley)
					topBlock=topBlock.union(bb.get("add"))
					if(linkIndex!=2)
						topBlock=topBlock
								.difference(topBottomBlockCutout.movez(rodlen+rodEmbedlen/2+sideBraceDistacne/2-bracing))
								.difference(backBoard)
					vitamins.addAll(bb.get("vitamins"))
					back.addAll(bb.get("vitamins"))
					pulley.setManipulator(frameListener)
					pulley.addAssemblyStep( 8+stepOffset, topVitaminsMove)
					for(CSG c:bb.get("vitamins")) {
						c.setManipulator(frameListener)
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
					CSG bearingBlock = new Cube(bearingBlcokBearingSection*2,
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

					bearingBlock= moveDHValues(bearingBlock,kin.getDhLink(linkIndex))
							.difference(vitamins)
							.difference(clearenceParts)

					makeLink2( back,    bearingBlock, kin,  linkIndex);
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
				def nutsAndBolts=[]
				def newBraceBlocks=[]
				for(CSG c:braceBlocks) {
					double sliceTHick=1
					CSG backPlate =c.intersect(c.getBoundingBox().toXMax().movex(c.getMinX()+sliceTHick))
					CSG frontPlate =c.intersect(c.getBoundingBox().toXMin().movex(c.getMaxX()-sliceTHick))
					boolean left = c.getMinY()<0
					CSG side
					double staage = linkIndex==0?0:0
					
					CSG sideSlice = new Cube(c.getTotalX(),sliceTHick,c.getTotalZ()).toCSG()
									.toXMin()
									.toZMin()
									.movex(c.getMinX())
									.movez(c.getMinZ())
					double moveAwayFromPulley =c.getMinZ()>rodlen/2?0:xyOfPulleyDistance+5
					
					if(!left)
						side = c.intersect(sideSlice.toYMin().movey(shaftHolderY/2-sliceTHick-staage-moveAwayFromPulley))
							.movey(moveAwayFromPulley)
					else
						side =c.intersect(sideSlice.toYMax().movey(-shaftHolderY/2+sliceTHick+staage+moveAwayFromPulley))
							.movey(-moveAwayFromPulley)
					newBraceBlocks.addAll([backPlate,frontPlate,side])
					double cornerBoltInset = 22
					Transform upperRight = new Transform()
							.move(backPlate.getMinX(),
							backPlate.getMinY()+cornerBoltInset,
							backPlate.getMaxZ()-cornerBoltInset)
					Transform upperLeft = new Transform()
							.move(backPlate.getMinX(),
							backPlate.getMaxY()-cornerBoltInset,
							backPlate.getMaxZ()-cornerBoltInset)
					Transform lowerRight = new Transform()
							.move(backPlate.getMinX(),
							backPlate.getMinY()+cornerBoltInset,
							backPlate.getMinZ()+cornerBoltInset)
					Transform lowerLeft = new Transform()
							.move(backPlate.getMinX(),
							backPlate.getMaxY()-cornerBoltInset,
							backPlate.getMinZ()+cornerBoltInset)

					Transform frontLeft = new Transform()
							.move(frontPlate.getMaxX(),
							frontPlate.getMaxY()-cornerBoltInset,
							frontPlate.getMinZ()+cornerBoltInset)
					Transform frontRight = new Transform()
							.move(frontPlate.getMaxX(),
							frontPlate.getMinY()+cornerBoltInset,
							frontPlate.getMaxZ()-cornerBoltInset)
							
					Transform sideTopBack = new Transform()
							.move(side.getMinX()+cornerBoltInset,
							left?side.getMinY():side.getMaxY(),
							side.getMaxZ()-cornerBoltInset)
					Transform sideBottomBack = new Transform()
							.move(side.getMinX()+cornerBoltInset,
							left?side.getMinY():side.getMaxY(),
							side.getMinZ()+cornerBoltInset)
					Transform sideBottomFront = new Transform()
							.move(side.getMaxX()-cornerBoltInset,
							left?side.getMinY():side.getMaxY(),
							side.getMinZ()+cornerBoltInset)
					Transform sideTopFront = new Transform()
							.move(side.getMaxX()-cornerBoltInset,
							left?side.getMinY():side.getMaxY(),
							side.getMaxZ()-cornerBoltInset)
					def boltLocations =[
						upperRight,
						upperLeft,
						lowerRight,
						lowerLeft,
						frontLeft,
						frontRight
						]
					if(linkIndex==0) {
						boltLocations.addAll([sideTopBack,sideBottomBack])
						if(c.getMinZ()>rodlen/2) {
							boltLocations.addAll([sideTopFront])
							
						}else
							boltLocations.addAll([sideBottomFront])
							
					}
					def myBits=[]
					for(Transform tf:boltLocations) {
						double angle=90
						double pullnut=-30
						double pullBolt=-300
						boolean isASide =false
						double rotX = 0
						double nutX =0
						if(tf.getX()>0) {
							angle=-angle
							pullnut=30
							pullBolt=300
						}
						//if(linkIndex==0) {
							if(tf.getY()>(shaftHolderY/2-sliceTHick)) {
								angle=180
								pullnut=30
								pullBolt=300
								rotX=90
								nutX=-90
								isASide=true
							}
							if(tf.getY()<(-shaftHolderY/2+sliceTHick)) {
								angle=180
								pullnut=-30
								pullBolt=-300
								rotX=-90
								nutX=90
								isASide=true
							}
						//}
						CSG myBolt = boltPulley.movez(boardThickness).rotx(rotX).roty(angle).transformed(tf)
						CSG myNutt = insert.rotx(nutX).roty((angle-180)%360).transformed(tf)
						if(!isASide) {
							myNutt.addAssemblyStep(1, new Transform().movex(pullnut))
							myBolt.addAssemblyStep(10, new Transform().movex(pullBolt))
						}else {
							myNutt.addAssemblyStep(1, new Transform().movey(pullnut))
							myBolt.addAssemblyStep(10, new Transform().movey(pullBolt))
						}
						if(c.getMaxZ()<rodlen/2) {
							myNutt.addAssemblyStep( 8+stepOffset, blockXAssembly)
							myNutt.addAssemblyStep( 9+stepOffset, blocZAssembly)
							myNutt.addAssemblyStep( 2, bottomSwing)
						}else{
							myNutt.addAssemblyStep( 8+stepOffset, topVitaminsMove)
							myNutt.addAssemblyStep( 2, topSplay )
						}
						backBoard=backBoard.difference(myBolt)
						frontBoard=frontBoard.difference(myBolt)
						for(int i=0;i<SideBoards.size();i++) {
							SideBoards.set(i, SideBoards.get(i).difference(myBolt))
						}
						myBits.add(myBolt)
						myBits.add(myNutt)
					}
					nutsAndBolts.addAll(myBits)
					newBraceBlocks.add(c.difference(myBits))
				}
				for(CSG c:nutsAndBolts) {
					c.setManipulator(frameListener)
				}

				back.addAll(nutsAndBolts)
				vitamins.addAll(nutsAndBolts)
				int blockIndex=1
				for(CSG c:newBraceBlocks) {
					if(c.getMaxZ()<rodlen/2) {
						c.addAssemblyStep( 8+stepOffset, blockXAssembly)
						c.addAssemblyStep( 9+stepOffset, blocZAssembly)
						c.addAssemblyStep( 2, bottomSwing)
					}else{
						c.addAssemblyStep( 8+stepOffset, topVitaminsMove)
						c.addAssemblyStep( 2, topSplay )
					}
					c.setManipulator(frameListener)
					if(linkIndex==0)
						c.setColor(Color.web("#4FC3C8"))
					if(linkIndex==1)
						c.setColor(Color.web("#00ff00"))
					if(linkIndex==2)
						c.setColor(Color.web("#ff00ff"))
					back.add(c)
					c.setName("Bracing-Block-"+blockIndex+"-link-"+linkIndex)
					blockIndex++
				}
				for(CSG c:vitamins) {
					c.setColor(Color.SILVER)
					c.setMfg({incoming->return null})
				}
				def boards = [backBoard]
				boards.addAll(SideBoards)
				if(linkIndex!=2) {
					CSG box = frontBoard.getBoundingBox().toYMin()
					CSG left = frontBoard.intersect(box)
					CSG right = frontBoard.difference(box)
					boards.addAll([left,right])
				}else {
					boards.add(frontBoard)
				}
				
				int boardIndex=1
				for(CSG c:boards) {
					if(c.getMaxX()>0) {
						c.addAssemblyStep( 9, new Transform().movex(braceHeight))
						c.addAssemblyStep( 4, new Transform().movez(rodlen+sideBraceDistacne*2))
					}else {
						c.addAssemblyStep( 4, new Transform().movez(rodlen+sideBraceDistacne*2))
						
					}
					if(c.getTotalX()-0.1<boardThickness)
						if(c.getMaxX()>0)
							c.setMfg({incoming->return incoming.roty(-90).toZMin()})
						else
							c.setMfg({incoming->return incoming.roty(90).toZMin()})

					if(c.getTotalY()-0.1<boardThickness)
						if(c.getMaxY()>0)
							c.setMfg({incoming->return incoming.rotx(90).toZMin()})
						else
							c.setMfg({incoming->return incoming.rotx(-90).toZMin()})
					if(c.getName().length()==0)
						c.setName("Board-"+boardIndex+"-link-"+linkIndex)
					boardIndex++
					c.setManipulator(frameListener)
					c.setColor(Color.web("#EDCAA1"))
					c.addExportFormat("svg")
					back.add(c)
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
			private void makeLink2(ArrayList<CSG> back,CSG bearingBlock,DHParameterKinematics kin, int linkIndex) {
				double cleatWidth = connectingBlockWidth

				CSG cleat = ((CSG)ScriptingEngine.gitScriptRun("https://github.com/TechnocopiaPlant/ForkyRobot.git", "cleat.groovy", [cleatWidth, cleatBracing]))
				.movey(-cleatWidth/2)
				double cleatDepth = cleat.getTotalX()

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
						.toZMin()
						.toXMin()
						.movez(-kinematicsToBottomOfBucket)


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
				pulley.setName("EOAT-Pulley")
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
				bucket.setName("bucketItself")
				back.add(bucketCleat)
				back.add(liftCleat)
				liftCleat.setName("EOAT-LiftCleat")
				bucketCleat.setName("bucket-LiftCleat")
				bucket.addAssemblyStep( 16, new Transform().movez(bucketHeightCentering*2))
				bucketCleat.addAssemblyStep( 16, new Transform().movez(bucketHeightCentering*2))
				bucket.addAssemblyStep(15, new Transform().movex(bucketHeightCentering*2))
				liftCleat.addAssemblyStep( 2, liftCleatAssembly)

			}
			@Override
			public ArrayList<CSG> generateBody(MobileBase arg0) {
				
				
				arg0.getAllDHChains().get(0).setRobotToFiducialTransform(baseOfArmFromCenter)
				HashMap<String,ArrayList<CSG>> bb =pulleyGen(new Transform())
				def back =[]
				//back.addAll(bb.get("add"))
				//back.addAll(bb.get("cut"))
				//back.addAll(bb.get("vitamins"))
				//				back.add(new Cube(1).toCSG())
				//				for(CSG c:back) {
				//					c.setManipulator(arg0.getRootListener())
				//					c.setMfg({inc->return null})
				//				}
				def baseParts=ScriptingEngine.gitScriptRun(
						HTTPS_GITHUB_COM_TECHNOCOPIA_PLANT_FORKY_ROBOT_GIT,
						"robotBase.groovy",null)
				for(CSG c:baseParts) {
					c.addAssemblyStep(15, new Transform().movex(-wheelbase*1.3))
				}
				back.addAll(baseParts)
				return back;
			}


		}