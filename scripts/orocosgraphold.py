# by Sjoerd van den Dries
# Expanded by Tim Clephas
# March 3, 2011

import sys, os

f_in = open(sys.argv[1])

components = []
subs = []
connections = []

substructs = {}

components.append("ROS")

for line in f_in:
    if line[0] != "#":
        if 'loadComponent' in line:
            s = line.split("\"")
            components.append(s[1])
            print s[1]
        elif 'connect' in line:
            s = line.split("\"")
            comp_sub1 = s[1]
            comp_sub2 = s[3]
            if '#' in line:
                s = line.split("#")
                port = s[-1].strip()
            else:
                port = ""
            comp1 = comp_sub1.split(".")[0]
            comp2 = comp_sub2.split(".")[0]
            subs.append(comp_sub1)
            subs.append(comp_sub2)
            
            if not comp1 in substructs:
                substructs[comp1] = []
            if not comp2 in substructs:
                substructs[comp2] = []
                          
            connections.append((comp1, comp2, port))
        elif 'stream' in line:
            s = line.split("\"")
            string = s[1]
            comp_port = string.split(".")
            
            comp = comp_port[0]
            port = "/" + comp_port[1]

            if '#in' in line:
                connections.append(("ROS", comp, port))
            else:
                connections.append((comp, "ROS", port))
            
f_in.close()

f_out = open(sys.argv[1].split(".")[0] + ".gv", 'w')

f_out.write('''// This file was auto-generated. Please do not edit.
digraph AMIGO_Software_Architecture {
    //rankdir = "LR"\n
    //ranksep=1.25\n
    //compound=true\n
    //ratio=fill\n
    //orientation = land\n
    //size = "11.69,8.27"\n // A4
    ''')



f_out.write('''\nnode [shape=box];\n
      "ROS"''')
f_out.write('''\nnode [shape=oval];\n''')

for comp in components:
    f_out.write('   \"' + comp + '\"' + '\n')

i = 0    

    
for (n1, n2, n3) in connections:
   f_out.write('   \"' + n1 + '\" -> \"' + n2 + '\"  [ label = \"' + n3 + '\" ]; \n')

f_out.write('}')
f_out.close()


