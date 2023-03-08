import com.google.gson.reflect.TypeToken

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cube
import eu.mihosoft.vrl.v3d.Wedge
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator
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

String HTTPS_GITHUB_COM_TECHNOCOPIA_PLANT_FORKY_ROBOT_GIT = "https://github.com/TechnocopiaPlant/ForkyRobot.git"
Type TT_mapStringString = new TypeToken<HashMap<String, Double>>() {}.getType();
Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();
File	cachejson = ScriptingEngine.fileFromGit(HTTPS_GITHUB_COM_TECHNOCOPIA_PLANT_FORKY_ROBOT_GIT, "baseDim.json")
def inPut = FileUtils.openInputStream(cachejson);
def jsonString = IOUtils.toString(inPut);
HashMap<String, Double> database = gson.fromJson(jsonString, TT_mapStringString);

double cleatDepthVal = database.cleatDepth
double depth = cleatDepthVal


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