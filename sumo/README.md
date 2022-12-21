# Simple underground parking level

## Fact Sheet
* Author: Marc Zofka
* Date: 2019-09-213
* SUMO 1.2

## Tutorial: Start the scenario
The simple underground parking level can be started this way
```bash
sumo-gui -c simple_underground_parking_level.sumocfg 
```

## Tutorial: Creating the road network and scenarios files manually
SUMO supports the manual definition of road networks. Therefore, plain xml files are meant to be edited. In order to define a network,
two files are needed: one file for nodes (=junctions), simple_underground_parking_level.nod.xml, and edges (=streets), simple_underground_parking_level.edg.xml.

### 1. Definition of Nodes
The nodes of the network are defined in the file simple_underground_parking_level.nod.xml:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<nodes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/nodes_file.xsd">

  <node id="01" x="41.4" y="52.9" type="priority"/>
  [...]
</nodes>
```
If the nodes are changed manually in this file, the road network has to re-compiled by the use of netconvert.

### 2. Definition of Edges
The edges of the network are to be defined in the file simple_underground_parking_level.nod.xml:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<edges xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/edges_file.xsd">

  <edge id="e_01_02" from="01" to="02" priority="2" numLanes="1" speed="11.11" spreadType="center" width="3.6"/>
  [...]
</edges>
```
If the edges of the road network are changed manually, the road network has to re-compiled by the use of netconvert.

### 3.  Netconvert: Creating a road network
After the files have been generated manually as described above, netconvert can be called this way:
```bash
netconvert --node-files=simple_underground_parking_level.nod.xml --edge-files=simple_underground_parking_level.edg.xml --output-file=simple_underground_parking_level.net.xml --verbose TRUE --error-log log/error.log --message-log log/messages.log --output.street-names TRUE --offset.disable-normalization TRUE --no-turnarounds true --junctions.corner-detail 10 --junctions.internal-link-detail 10 --rectangular-lane-cut TRUE --walkingareas TRUE --write-license TRUE --save-configuration simple_underground_parking_level.netccfg
```
The following parameters are used in the program call above and can be modified for experiences. More parameters can be found in the [SUMO's Netconvert Options](https://sumo.dlr.de/docs/NETCONVERT.html).

| Parameter        | Explanation    |  Comment
| ------------------- |:-----------------:|:---------------|
|--output.street-names BOOL                  | Street names will be included in the output (if available); default: false | to be tested
| -- offset.disable-normalization BOOL    | Turn off normalizing node positions; default: false                                        | assures that coordinate system remain constant during netconvert
| -- junctions.corner-detail INT                 | Generate INT intermediate points to smooth out intersection corners; default: 5            | to be tested
| --junctions.internal-link-detail               | Generate INT intermediate points to smooth out lanes within the intersection; default: 5   | to be tested
| --save-configuration FILE                       | Saves current configuration into FILE                                                      |

The configuration file then can be applied in order to generate the road network:
```bash
netconvert -c simple_underground_parking_level.netccfg
```

The road network generation process should produce something like:
```bash
Loading configuration... done.
Parsing nodes from 'simple_underground_parking_level.nod.xml'... done.
Parsing edges from 'simple_underground_parking_level.edg.xml'... done.
 Import done:
   11 nodes loaded.
   12 edges loaded.
   
[...]

Building inner edges... 
Warning: Speed of straight connection 'e_01_02_0->e_02_10_0' reduced by 6.26 due to turning radius of 4.27 (length=1.04 angle=32.59)
done (1ms).
-----------------------------------------------------
Summary:
 Node type statistics:
  Unregulated junctions       : 0
  Dead-end junctions          : 2
  Priority junctions          : 9
  Right-before-left junctions : 0
 Network boundaries:
  Original boundary  : 10.00,2.25,44.80,52.90
  Applied offset     : 0.00,0.00
  Converted boundary : 10.00,2.25,44.80,52.90
-----------------------------------------------------
Writing network... done (2ms).
Success.
```

### 4. Create polygon file
In order to provide an aedequate visualization, a simple_underground_parking_level.poly.xml can be defined, containing polygons:
```xml
<?xml version="1.0" encoding="UTF-8"?>

<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">
   
    <poly id="1" type="reference_plane"   color="40,40,40"  fill="1" layer="1.0"  shape="0.0,0.0 54.8,0.0 54.8,55.1 0.0,55.1"/> 
    [...]
</additional>
```

### 5. Create parking areas
In order to define fixed parking areas and parking spaces, a parking_areas.add.xml has to be defined:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">
  <parkingArea id="Parking_Bay_Left" lane="e_11_12_0" startPos="0" endPos="30" roadsideCapacity="0" angle="90">
    <space x="6.5" y="48.25"  width="2.3"    length="5"/> 
    <space x="6.5" y="45.95"  width="2.3"    length="5"/>
    [...]
</additional>
```
More information about simulation of Parking Areas can be found under the [SUMO Wiki](https://sumo.dlr.de/docs/Simulation/ParkingArea.html).

### 6. Define a scenario set by combining all together
In other to define fixed scenario, a *.sumocfg file has to be created, which relates all created aspects to a common and unified scenario:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="simple_underground_parking_level.net.xml"/>
        <additional-files value="simple_underground_parking_level.poly.xml,parking_areas.add.xml"/>
        <route-files value="simple_underground_parking_level.rou.xml"/>
    </input>
    <gui_only>
        <gui-settings-file value="gui_config_default.xml"/>  
        <gui-testing value="true"/>
        <gui-testing-debug value="true"/>
    </gui_only>
    <processing>
        <lateral-resolution value="0.2"/>  
        <collision.action value="warn"/>
        <collision.check-junctions value="true"/>
        <threads value="1"/> <!-- 2 results in instable behavior-->
        <!--<pedestrian.model value="nonInteracting"/>-->
    </processing>
    <time>
        <begin value="0"/>
        <end value="3600"/>
        <step-length value=".05"/>
    </time>
    <traci_server>
        <!--<remote-port value="8311"/>-->
    </traci_server>
    <report>
        <verbose value="true"/>
        <no-step-log value="true"/>
        <log value="/tmp/my_log.dat"/>
    </report>
</configuration>
```

## Tutorial: Inspect the road network with netedit
The road network can be analyzed as well as modified with the netedit tool. In order to operate on the basic files, the nodes and edge files have to be given as parameters this way:
```bash
netedit --node-files simple_underground_parking_level.nod.xml --edge-files simple_underground_parking_level.edg.xml --additional-files parking_areas.add.xml,simple_underground_parking_level.poly.xml --verbose
```
