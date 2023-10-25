//
// Created by Francisco Munoz on 19/06/19.
/*这段代码是一个并行计算中用到的降维树的C++实现。降维树是一种二叉树，每个节点对其两个子节点执行降维操作。代码创建了一个类似于论文中所示的降维树。
FENetwork类的构造函数接受id_t id、std::string name、Config stonne_cfg和Connection* outputConnection。Config对象用于从配置文件中收集参数。port_width和ms_size参数用于确保乘法器的数量是2的幂。nlevels变量用于存储树的层数。
代码从根节点开始创建树的节点，从上到下移动到叶子节点。每个节点都是一个FEASwitch对象，它是Switch的子类。FEASwitch构造函数接受id_t id、std::string name、int level、int num和Config stonne_cfg。level和num参数用于确定节点在树中的位置。stonne_cfg参数用于设置节点的配置。
代码然后连接树的节点。第一个节点连接到outputConnection。其他节点连接到上一级的输出连接。代码还创建了节点之间的转发连接。转发连接用于连接没有共同父节点的节点。
代码还创建了Connection对象，用于将节点连接到内存并创建折叠链接。折叠链接用于连接共享父节点的节点。
总的来说，代码使用FEASwitch对象创建了一个降维树，并使用Connection对象将它们连接起来*/
#include "FENetwork.h"
#include <assert.h>
#include "utility.h"
#include <math.h>

//TODO Conectar los enlaces intermedios de forwarding
//This Constructor creates the reduction tree similar to the one shown in the paper
FENetwork::FENetwork(id_t id, std::string name, Config stonne_cfg, Connection* outputConnection) : ReduceNetwork(id, name) {
    // Collecting the parameters from configuration file
    this->port_width=stonne_cfg.m_ASwitchCfg.port_width;
    this->ms_size = stonne_cfg.m_MSNetworkCfg.ms_size;
    stonne_cfg.m_ASNetworkCfg.accumulation_buffer_enabled=0; //Always 0 in this parameter, ignoring
    //End collecting the parameters from the configuration file
    assert(ispowerof2(ms_size)); //Ensure the number of multipliers is power of 2.
    this->outputConnection = outputConnection;
    int nlevels = log10(ms_size) / log10(2); //All the levels without count the leaves (MSwitches)
    this->nlevels = nlevels;
    int adders_this_level = 2;
    unsigned int as_id = 0;
	//计算出缩减树的层数，并在每一层上创建ASwitch对象。在创建ASwitch对象时，它会为每个ASwitch对象分配一个唯一的ID，并将其添加到一个名为aswitchtable的映射表中。此外，它还会为每个ASwitch对象创建输入连接和输出连接
    for(int i=0; i < this->nlevels; i++) {  //From root to leaves (without the MSs)
        for(int j=0; j < adders_this_level; j++) { // From left to right of the structure
            std::string as_str="FEASwitch "+as_id;
            FEASwitch* as = new FEASwitch(as_id, as_str, i, j, stonne_cfg);
            as_id+=1; //increasing the as id 
            std::pair<int, int> levelandnum (i,j);
            aswitchtable[levelandnum] = as;
           
            /* Now its the bus who create this  
            //Connecting with output memory. 
            //Creating the connection between this adder and the collectionBus
            Connection* memoryConnection = new Connection(output_ports);
            as->setMemoryConnection(memoryConnection);
            memoryconnectiontable[levelandnum]=memoryConnection;
            */

    
            if(i==0) {  //The first node is connected to the outputConnection. 
                if(j==0) {
                    as->setOutputConnection(outputConnection);
                    this->single_switches.push_back(as); //The first node is a single reduction switch since it has no forwarding links.
                    Connection* folding_connection = new Connection(port_width);
                    foldingconnectiontable[levelandnum]=folding_connection;
                    as->setUpNodeForwardingConnection(folding_connection);
                }
                if(j==1) {
                std::pair<int,int> levelandnumroot(0,0);
                Connection* input_folding_connection = foldingconnectiontable[levelandnumroot];
                this->single_switches.push_back(as);
                as->addInputConnectionToForward(input_folding_connection);
                
                }
                
            }

            else { //Connecting level i nodes with level -1 connections that are the output connections of the previous level. THere are the same number of output connections and nodes. 
            	//父子间连接(由于是双向连接，上一层创建的两个向下连接，这一层来连)
                std::pair<int, int> sourcepair(i-1, j); // the same number of connections in the above level matches the number of Dswitches in this l
                Connection* correspondingOutputConnection = inputconnectiontable[sourcepair]; //output node i is the input of the node -i, already connection in the previous loop (code below)
                as->setOutputConnection(correspondingOutputConnection); 

				//中间横向连接
                //Connecting forwarding connections (intermedium links). Even (par) node with level>0 and node>0) is the second second node of a node with fw connection.
                if(j && ((j % 2) == 0)) { //FW link is needed. Connecting the consecutive node that does not share parent with the previous one
                    //Creating the fw link
                    Connection* fwconnection = new Connection(port_width);
                    //Adding the fw to the map. The index is the second node of the pair
                    std::pair<int, int> fwpair(i, j);
                    forwardingconnectiontable[fwpair]=fwconnection;
                    //Connecting fw link to the current one and the previous node, which has no shared parent
                    as->setForwardingConnection(fwconnection);
                    std::pair<int, int> previous_pair (i, j-1); 
                    FEASwitch* previous_as = aswitchtable[previous_pair];
                    previous_as->setForwardingConnection(fwconnection); //Connected with the same fwlink
                    //Inserting to the list double reduction switch. We insert both as double but actually both are an unit
                    this->double_switches.push_back(previous_as);
                    this->double_switches.push_back(as);
                    
                    
                    
                }
				//每一层的第一个和最后一个AS是没有横向fw link的，把他加入single_switches中（虽然现在还不知道加进去有啥用）
               else if((j==0) || (j==(adders_this_level-1))) { //If it is the first or the last as then is a single reduction switch
                   this->single_switches.push_back(as);
               }
			   
				//STIFT论文中额外增加的fw link---也就是folding_connection（比ART树多的函数setUpNodeForwardingConnection和addInputConnectionToForward，一个是结点向上伸出的，一个是结点接收的）
               //Advanced code to determine where the folding link is added
               Connection* folding_connection = new Connection(port_width);//除根节点外所有节点都先创一个向上的folding_connect，而输入链接可以有多个所以用一个vector存放
               foldingconnectiontable[levelandnum]=folding_connection;
               as->setUpNodeForwardingConnection(folding_connection);
			   
               //Calculating the other side of the link ，连接的接收端！
               //论文中粉色箭头
               if((j % 2) == 0) { //IF it is even偶数 then the connection is to the node that is the direct parent
                   unsigned int fw_as_level = i-1;
                   unsigned int fw_num = j / 2;
                   std::pair<int,int> as_fw_id(fw_as_level, fw_num);
                   FEASwitch* as_fw = aswitchtable[as_fw_id];
                   as_fw->addInputConnectionToForward(folding_connection);//为他亲爸增加一个输入链接，论文中粉色箭头
               }
				//论文中红色箭头
               else { //If not two cases: connection to the next parent that turn right or connection to the special node
                   unsigned int n_reducciones=0; 
                   unsigned int current_node = j;
                   while((current_node % 2) != 0) { //Mientras que current_node sea impar
                       n_reducciones++;
                       current_node = current_node / 2; //Trunca
                   
                   }

                   int level_node_to_reduce = i - n_reducciones - 1;
                   if(level_node_to_reduce < 0 ) { //Special node
                       std::pair<int,int> as_fw_id(0, 1);//连到第二个根节点，论文中是连到第二个根节点的右边三个红箭头
                       FEASwitch* as_fw = aswitchtable[as_fw_id];
                       as_fw->addInputConnectionToForward(folding_connection);
                   }

                   else { //The next parent that turn right（比如n_reducciones=1的，就连它爷爷，n_reducciones=2，就连他太爷爷）
                       unsigned int node_to_reduce = current_node / 2; //Padre del que da par 
                       std::pair<int,int> as_fw_id(level_node_to_reduce, node_to_reduce);
                       FEASwitch* as_fw = aswitchtable[as_fw_id];
                       as_fw->addInputConnectionToForward(folding_connection);

                   }
               }


              //Connecting link

            }
    
            //Creating and Connecting input connections (left and right)
            //For each connection of this particular node
            for(int c=0; c < CONNECTIONS_PER_SWITCH; c++) {
                Connection* connection = new Connection(port_width); //Output link so output ports
                int connection_pos_this_level = j*CONNECTIONS_PER_SWITCH+c; //number of switches alreay created + shift this switch
                std::pair<int, int> connectionpair (i, connection_pos_this_level);
                inputconnectiontable[connectionpair] = connection;
                //Connecting adder with its input connections
                if(c == LEFT) {
                    as->setInputLeftConnection(connection);
                }

                else if(c == RIGHT) {
                     as->setInputRightConnection(connection);
                }

            }

          
        
        }
        //In the next level, the input ports is the output ports of this level
        if(i>0) {
            adders_this_level=adders_this_level * 2; 
        
        }
    }



}


void FENetwork::setMemoryConnections(std::vector<std::vector<Connection*>>  memoryConnections) {
     unsigned int n_bus_lines = memoryConnections.size();
     std::cout << "N_bus_lines: " << n_bus_lines << std::endl;
    // Interconnect double reduction switches
    for(int i=0; i<double_switches.size(); i+=2) { //2 by 2 since each 2 aswitches form a single double switch
        unsigned int inputID = 2* ((i/2) /  n_bus_lines);
        unsigned int busID = (i/2) % n_bus_lines;
        FEASwitch* as_first = double_switches[i];
        FEASwitch* as_second = double_switches[i+1];
        assert(busID < memoryConnections.size()); //Making sure the CollectionBus returns the correct busLine
        assert((inputID+1) < memoryConnections[busID].size()); //Making sure the CollectionBus returns the correct busLine
        std::cout << "SIZE: " << memoryConnections[busID].size() << std::endl;
        Connection* conn_first = memoryConnections[busID][inputID]; //Connecting as i to busID, inputID connection
        Connection* conn_second = memoryConnections[busID][inputID+1];
        as_first->setMemoryConnection(conn_first, busID, inputID);
        as_second->setMemoryConnection(conn_second, busID, inputID+1);
        std::cout << "FEASwitch " << as_first->getLevel() << ":" << as_first->getNumInLevel() << " connected to BUS " << busID << " INPUT " << inputID << std::endl;
        std::cout << "FEASwitch " << as_second->getLevel() << ":" << as_second->getNumInLevel() << " connected to BUS " << busID << " INPUT " << inputID+1 << std::endl;



    }

    for(int i=0; i<single_switches.size(); i++) {
        unsigned int inputID_Base = (double_switches.size() / n_bus_lines) + 1; //Noice that here we do not use 2*double_switches since there are single switches in the list considered as double
        unsigned  int inputID = inputID_Base + (i / n_bus_lines);
        unsigned int busID = (i+double_switches.size()) % n_bus_lines;
        FEASwitch* as = single_switches[i];
        Connection* conn = memoryConnections[busID][inputID];
        as->setMemoryConnection(conn, busID, inputID);
        std::cout << "FEASwitch " << as->getLevel() << ":" << as->getNumInLevel() << " connected to BUS " << busID << " INPUT " << inputID << std::endl;


    }


}

FENetwork::~FENetwork() {
    //Delete the adders from aswitchtable
    for(std::map<std::pair<int, int>, FEASwitch*>::iterator it=aswitchtable.begin(); it != aswitchtable.end(); ++it) {
        delete it->second; 
    }
    
    //Removing connections from inputconnectiontable
    for(std::map<std::pair<int, int>, Connection*>::iterator it=inputconnectiontable.begin(); it != inputconnectiontable.end(); ++it) {
        delete it->second;
    }

    //Delete forwarding connections
    for(std::map<std::pair<int, int>, Connection*>::iterator it=forwardingconnectiontable.begin(); it != forwardingconnectiontable.end(); ++it) {
        delete it->second;
    }

    //Delete folding connections
    for(std::map<std::pair<int, int>, Connection*>::iterator it=foldingconnectiontable.begin(); it != foldingconnectiontable.end(); ++it) {
        delete it->second;
    }



}

std::map<int, Connection*> FENetwork::getLastLevelConnections() {
    int last_level_index = this->nlevels-1; //The levels start from 0, so in the table the last level is nlevels-1
    std::map<int, Connection*> connectionsLastLevel; //Map with the connections of the last level of AS (the ones that must be cnnected with the MN)
    for(int i=0; i<this->ms_size; i++) {  //Each multiplier must have its own connection if the ASNetwork has been created correctly
        std::pair<int,int> current_connection (last_level_index, i);
        connectionsLastLevel[i]=this->inputconnectiontable[current_connection];
    }
    return connectionsLastLevel;
}

/*
    This function set a different configuration for each switch of the network. Each pair i, j corresponds to the level and the id in the
    level for each switch. Each pair has a correspondency with a configuration (ADD_2_1, ADD_3_1, ..., etc.)
*/
void FENetwork::addersConfiguration(std::map<std::pair<int,int>, adderconfig_t> adder_configurations) {
  /* This code is for iterating over all the switches. Forcing the user to set a configuration for each switch (when maybe there is some switch that does not have to)
    int switches_this_level = 1;
    for(int i=0; i< nlevels; i++) {
        for(int j=0; j < switches_this_level; j++) {
            std::pair<int, int> current_switch_pair(i,j);
            FEASwitch* as = aswitchtable[current_switch_pair];
            adderconfig_t conf = adder_configurations[current_switch_pair]; //Getting the configuration for that specific adder
            as->setConfigurationMode(conf);
            //Setting

            
        }
        switches_this_level=switches_this_level*2;
    }  
   */
    for(std::map<std::pair<int,int>, adderconfig_t>::iterator it=adder_configurations.begin(); it != adder_configurations.end(); ++it) {
        adderconfig_t conf = it->second;
        FEASwitch* as = aswitchtable[it->first]; //pair index
        as->setConfigurationMode(conf);
    }
}

void FENetwork::forwardingConfiguration(std::map<std::pair<int,int>, fl_t> fl_configurations) {
    for(std::map<std::pair<int,int>,fl_t>::iterator it=fl_configurations.begin(); it != fl_configurations.end(); ++it) {
        FEASwitch* as = aswitchtable[it->first]; //Pair index
        as->setForwardingLinkDirection(it->second); //Setting the direction
    }
}

void FENetwork::childsLinksConfiguration(std::map<std::pair<int,int>, std::pair<bool,bool>> childs_configuration) {
    for(std::map<std::pair<int,int>, std::pair<bool,bool>>::iterator it=childs_configuration.begin(); it != childs_configuration.end(); ++it)
    {
        FEASwitch* as = aswitchtable[it->first];
        bool left_child_enabled = std::get<0>(it->second);
        bool right_child_enabled = std::get<1>(it->second);
        as->setChildsEnabled(left_child_enabled, right_child_enabled);
   } 
}

void FENetwork::forwardingToMemoryConfiguration(std::map<std::pair<int,int>, bool> forwarding_to_memory_enabled) {
    for(std::map<std::pair<int,int>, bool>::iterator it=forwarding_to_memory_enabled.begin(); it != forwarding_to_memory_enabled.end(); ++it) {
        FEASwitch* as = aswitchtable[it->first];
        bool forwarding_to_memory = it->second;
        as->setForwardingToMemoryEnabled(forwarding_to_memory);
    }
}

void FENetwork::configureSignals(Tile* current_tile, DNNLayer* dnn_layer, unsigned int ms_size, unsigned int n_folding) {
    CompilerFEN* compiler_fen = new CompilerFEN(); //Creating the object
    compiler_fen->configureSignals(current_tile, dnn_layer, ms_size, n_folding);  
    std::map<std::pair<int,int>,adderconfig_t> as_signals = compiler_fen->get_switches_configuration();
    std::map<std::pair<int,int>, fl_t> as_fw_signals = compiler_fen->get_fwlinks_configuration();
    std::map<std::pair<int,int>, std::pair<bool,bool>> as_childs_enabled = compiler_fen->get_childs_enabled();
    std::map<std::pair<int,int>, bool> forwarding_to_memory_enabled = compiler_fen->get_forwarding_to_memory_enabled();
    std::map<std::pair<int,int>, bool> forwarding_to_fold_node_enabled = compiler_fen->get_forwarding_to_fold_node_enabled();
    //unsigned int n_folding = (dnn_layer->get_R() / current_tile->get_T_R())*(dnn_layer->get_S() / current_tile->get_T_S()) * (dnn_layer->get_C() / current_tile->get_T_C()) ;
    this->addersConfiguration(as_signals);
    this->forwardingConfiguration(as_fw_signals);
    this->childsLinksConfiguration(as_childs_enabled);
    this->forwardingToMemoryConfiguration(forwarding_to_memory_enabled);
    this->forwardingToFoldNodeConfiguration(forwarding_to_fold_node_enabled);
    this->NPSumsConfiguration(n_folding); //All the nodes with the same folding configuration
    delete compiler_fen;

} 

void FENetwork::NPSumsConfiguration(unsigned int n_psums) {
     for(std::map<std::pair<int, int>, FEASwitch*>::iterator it=aswitchtable.begin(); it != aswitchtable.end(); ++it) {
        it->second->setNPSums(n_psums);
    }


}

void FENetwork::forwardingToFoldNodeConfiguration(std::map<std::pair<int,int>, bool> forwarding_to_fold_node_enabled) {
    for(std::map<std::pair<int,int>, bool>::iterator it=forwarding_to_fold_node_enabled.begin(); it != forwarding_to_fold_node_enabled.end(); ++it) {
        FEASwitch* as = aswitchtable[it->first];
        bool forwarding_to_fold_node = it->second;
        as->setForwardingToFoldNodeEnabled(forwarding_to_fold_node);
    }
}

//TODO Implementar esto bien
void FENetwork::cycle() {
    // First are executed the adders that receive the information. This way, the adders uses the data generated in the previous cycle.
    //The order from root to leaves is important in terms of the correctness of the network.
    int switches_this_level = 2; //Only one switch in the root
    //Going down to the leaves (no count the MSs)
    std::pair<int,int> current_switch_pair(0,0); //First node
    std::pair<int,int> extra_switch_pair(0,1);
    FEASwitch* as = aswitchtable[current_switch_pair];
    FEASwitch* as_extra = aswitchtable[extra_switch_pair];
    as->cycle();
    as_extra->cycle();
    for(int i=1; i < this->nlevels; i++) { //From level 1  
        int j = 0;
        while(j < switches_this_level) {
            std::pair<int,int> current_switch_pair (i,j);
            FEASwitch* as = aswitchtable[current_switch_pair]; //Current node j
            //Determining the order of execution if the forwarding link is enabled
            if(as->isFwEnabled()) { //If there is fw link in this node
                std::pair<int,int> current_switch_pair (i,j+1); //Getting the next which is connected witht his
                FEASwitch* as_next = aswitchtable[current_switch_pair];
                //Determining the order.
                if(as->getFlDirection() == SEND) { //If the current one is who send the data, the next goes first since it has to take the data in the next cycle
                    as_next->cycle();
                    as->cycle();  //executing the cycle for the current DS. 
                }
                else { //Direction == RECEIVE
                    as->cycle();
                    as_next->cycle();
                }
                j+=2; //skip the next one since it has been processed in this iteration

            } //End fw link is enabled
            else { //Just this node is executed. The next one and the order between the do not matter at all
                as->cycle();
                j++; //Not skip since the next one is not performed.
           }
        }
        switches_this_level = switches_this_level*2;    
    }

}

//Print configuration of the ASNetwork 
void FENetwork::printConfiguration(std::ofstream& out, unsigned int indent) {

    out << ind(indent) << "\"ASNetworkConfiguration\" : {" << std::endl;
        //out << ind(indent+IND_SIZE) << "\"ms_size\" : " << this->ms_size  << std::endl; DSNetwork global parameters
        out << ind(indent+IND_SIZE) << "\"ASwitchConfiguration\" : [" << std::endl;   //One entry per DSwitch
        int switches_this_level = 2;
        for(int i=0; i < this->nlevels; i++) {  //From root to leaves (without the MSs)
        //Calculating the output ports in this level
            //One array for each level will allow the access to the FEASwitch easier
            out << ind(indent+IND_SIZE+IND_SIZE) << "[" << std::endl;
            for(int j=0; j < switches_this_level; j++) { // From left to right of the structure
                std::pair<int,int> current_switch_pair (i,j);
                FEASwitch* as = aswitchtable[current_switch_pair];
                as->printConfiguration(out, indent+IND_SIZE+IND_SIZE+IND_SIZE);
                if(j==(switches_this_level-1)) {  //If I am in the last switch of the level, the comma to separate the swes is not added
                    out << std::endl; //This is added because the call to ds print do not show it (to be able to put the comma, if neccesary)
                }
                else {
                    out << "," << std::endl; //Comma and line break are added to separate with the next FEASwitch in the array of this level
                }

            }
            if(i==(this->nlevels-1)) { //If I am in the last level, the comma to separate the different levels is not added
                out << ind(indent+IND_SIZE+IND_SIZE) << "]" << std::endl;
            }

            else { //If I am not in the last level, then the comma is printed to separate with the next level
                out << ind(indent+IND_SIZE+IND_SIZE) << "]," << std::endl;
            }
             //In the next level, the input ports is the output ports of this level
            if(i>0) {
                switches_this_level=switches_this_level*2;
            }
        }


        out << ind(indent+IND_SIZE) << "]" << std::endl;
    out << ind(indent) << "}";
}

//Printing stats
void FENetwork::printStats(std::ofstream& out, unsigned int indent) {

    out << ind(indent) << "\"ASNetworkStats\" : {" << std::endl; 
        //out << ind(indent+IND_SIZE) << "\"ms_size\" : " << this->ms_size  << std::endl; DSNetwork global parameters
        out << ind(indent+IND_SIZE) << "\"ASwitchStats\" : [" << std::endl;   //One entry per DSwitch
        int switches_this_level = 2;
        for(int i=0; i < this->nlevels; i++) {  //From root to leaves (without the MSs)
        //Calculating the output ports in this level
            //One array for each level will allow the access to the FEASwitch easier
            out << ind(indent+IND_SIZE+IND_SIZE) << "[" << std::endl;
            for(int j=0; j < switches_this_level; j++) { // From left to right of the structure
                std::pair<int,int> current_switch_pair (i,j);
                FEASwitch* as = aswitchtable[current_switch_pair];
                as->printStats(out, indent+IND_SIZE+IND_SIZE+IND_SIZE);
                if(j==(switches_this_level-1)) {  //If I am in the last switch of the level, the comma to separate the swes is not added
                    out << std::endl; //This is added because the call to ds print do not show it (to be able to put the comma, if neccesary)
                }
                else {
                    out << "," << std::endl; //Comma and line break are added to separate with the next FEASwitch in the array of this level
                }

            }
            if(i==(this->nlevels-1)) { //If I am in the last level, the comma to separate the different levels is not added
                out << ind(indent+IND_SIZE+IND_SIZE) << "]" << std::endl;
            }

            else { //If I am not in the last level, then the comma is printed to separate with the next level
                out << ind(indent+IND_SIZE+IND_SIZE) << "]," << std::endl;
            }
             //In the next level, the input ports is the output ports of this level
            if(i>0) {
                switches_this_level=switches_this_level*2;
            }
        }


        out << ind(indent+IND_SIZE) << "]" << std::endl;
    out << ind(indent) << "}";
}

void FENetwork::printEnergy(std::ofstream& out, unsigned int indent) {
     /*
      The ASNetwork component prints the counters for the next subcomponents:
          - FEASwitches
          - wires that connect each aswitch with its childs (including the level of wires that connects with the MSNetwork)
          - augmented wires between adders
          - wires that connect each aswitch with its corresponding aswitch in charge of performing the partial sums

      Note that the wires that connect with memory are not taken into account in this component. This is done in the CollectionBus.

     */

     //Printing the input wires
     for(std::map<std::pair<int, int>, Connection*>::iterator it=inputconnectiontable.begin(); it != inputconnectiontable.end(); ++it) {
         Connection* conn = inputconnectiontable[it->first];
         conn->printEnergy(out, indent, "RN_WIRE");
     }

     for(std::map<std::pair<int, int>, Connection*>::iterator it=forwardingconnectiontable.begin(); it != forwardingconnectiontable.end(); ++it) {
         Connection* conn = forwardingconnectiontable[it->first];
         conn->printEnergy(out, indent, "RN_WIRE");
     }

     for(std::map<std::pair<int, int>, Connection*>::iterator it=foldingconnectiontable.begin(); it != foldingconnectiontable.end(); ++it) {
         Connection* conn = foldingconnectiontable[it->first];
         conn->printEnergy(out, indent, "RN_WIRE"); 
     }


    //Printing the FEASwitches energy stats and their fifos stats
     for(std::map<std::pair<int,int>,FEASwitch*>::iterator it=aswitchtable.begin(); it != aswitchtable.end(); ++it) {
        FEASwitch* as = aswitchtable[it->first]; //Pair index
        as->printEnergy(out, indent); //Setting the direction
     }

   

     

}
