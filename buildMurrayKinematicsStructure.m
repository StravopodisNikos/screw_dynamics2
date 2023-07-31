function [s_murray] = buildMurrayKinematicsStructure(gsai0,gst0,xi_ai)

field1 = 'gsai0'; value1 = gsai0; 
field2 = 'gst0'; value2 = gst0; 
field3 = 'xi_ai'; value3 = xi_ai; 
s_murray = struct(field1,value1,field2,value2,field3,value3);
end