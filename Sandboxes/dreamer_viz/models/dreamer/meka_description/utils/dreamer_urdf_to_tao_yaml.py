#!/usr/bin/env python
#import easy to use xml parser called minidom:
from xml.dom.minidom import parse, parseString 
 
#Parse the file
urdf = parse("../robots/dreamer.urdf")
ofile = open("dreamer_active_links.yaml","w")
s = '['
#Go through all link tags
for node in urdf.getElementsByTagName('link'):      
        a = node.attributes["name"]
        
        #if(a.value!='world'): #use to sort out inactive links (ie, eyes)
        s = s+'\''+a.value+'\''+','
        	#print a.value
              	        
        		

s = s[:-1]        
s = s+']'
print s
ofile.write(s);
ofile.close();

        
