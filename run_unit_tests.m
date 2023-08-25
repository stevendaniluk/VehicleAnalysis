init_vehicle_analysis_paths();
import matlab.unittest.TestSuite;

suiteFile = TestSuite.fromFile('Motec/MotecHandlerTest.m');
result = run(suiteFile);
