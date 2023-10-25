#include "SDMemory.h"
#include <assert.h>
#include <iostream>
#include "utility.h"
#include <math.h>

//The memory is ordered setting the channels in consecutive words in memory

VNAT_Register::VNAT_Register(unsigned int VN, unsigned int addr, unsigned int K, unsigned int X,  unsigned int iter_S, unsigned int iter_K, unsigned int iter_X,  DNNLayer* dnn_layer, Tile* current_tile) {
        this->VN = VN;
        this->base_addr = addr; //This address is always fixed since is the one use to make the calculation of next address easier.
        this->addr = addr; //This addr change over time
        this->current_S = 0;
        this->current_K = 0;
        this->current_X = 0;

        this->iter_K = iter_K;
        this->iter_X = iter_X;
        this->iter_S = iter_S;


	//Bases
	this->base_K=K;
	this->base_X=X;

        this->n_psums=iter_S;   //K
        this->current_psum=0;
        this->dnn_layer = dnn_layer;
        this->current_tile = current_tile;
	if((base_K < this->dnn_layer->get_K()) && (base_X < this->dnn_layer->get_X_()) ) {
	   this->valid_value = true; //Zero-remainder constraint
	}

	else {
            this->valid_value = false; //Zero-remainder constraint
	}
}

//Return the offset from 0
unsigned int VNAT_Register::get_address() {
    return this->base_addr + addr;
}


//TODO revisar esta funcion 
void VNAT_Register::update() {
        this->current_S+=1;
        if(this->current_S == this->iter_S) {
            this->current_S = 0;
            this->current_X+=1;
            // this->current_tile->get_T_X_()-1 because the address is already in next X iteration  (since it is consecutive to the previous Y)
            if(this->current_X == this->iter_X) {
                this->current_X = 0; //Updating X
                this->current_K+=1;
                if(this->current_K==this->iter_K) { //Go to next N
                    this->current_K = 0;          
                } //end iter_K
            } //end iter_X
     
        } //End S
        unsigned index_X=current_X*this->current_tile->get_T_X_();
        unsigned index_K=current_K*this->current_tile->get_T_K();


		//addr是偏移为啥要加上base_addr呢？
        this->addr = this->base_addr + index_X*dnn_layer->get_K()*word_size  + index_K*word_size;
	if( ((base_K+index_K) < this->dnn_layer->get_K()) && ((base_X+index_X) < this->dnn_layer->get_X_()) ) {
		this->valid_value = true;	
        //std::cout << "Address: " << this->addr << std::endl;
	}

	else {
            this->valid_value = false;
        }
  
    
}

SDMemory::SDMemory(id_t id, std::string name, Config stonne_cfg, Connection* write_connection) : MemoryController(id, name) {
    this->write_connection = write_connection;
    //Collecting parameters from the configuration file
    this->num_ms = stonne_cfg.m_MSNetworkCfg.ms_size;  //Used to send data
    this->n_read_ports=stonne_cfg.m_SDMemoryCfg.n_read_ports;
    this->n_write_ports=stonne_cfg.m_SDMemoryCfg.n_write_ports;
    this->write_buffer_capacity=stonne_cfg.m_SDMemoryCfg.write_buffer_capacity;
    this->port_width=stonne_cfg.m_SDMemoryCfg.port_width;
    //End collecting parameters from the configuration file
    //Initializing parameters
    this->ms_size_per_input_port = this->num_ms / this->n_read_ports;
    this->write_fifo = new Fifo(write_buffer_capacity);//一个写端口？
    for(int i=0; i<this->n_read_ports; i++) {//为每个读端口分配一个read_fi和一个psum_fi
        Fifo* read_fi = new Fifo(this->write_buffer_capacity);
        Fifo* psum_fi = new Fifo(this->write_buffer_capacity);
        input_fifos.push_back(read_fi);
        psum_fifos.push_back(psum_fi);
        this->sdmemoryStats.n_SRAM_read_ports_weights_use.push_back(0);  //To track information
        this->sdmemoryStats.n_SRAM_read_ports_inputs_use.push_back(0);   //To track information
        this->sdmemoryStats.n_SRAM_read_ports_psums_use.push_back(0);    //To track information
    }
    for(int i=0; i<this->n_write_ports; i++) {  //To track information
        this->sdmemoryStats.n_SRAM_write_ports_use.push_back(0);  //To track information
    }  //To track information
    this->weights_distributed = false;
    this->fw_link_enabled = false;
    this->weights_finished = false;
    this->input_finished = false;
    this->tile_loaded = false;
    this->execution_finished = false;
    this->current_output_pixel = 0;
    this->local_cycle=0;
    
    /* Now this is done by the bus
    //Creating the write ports
    for(int i=0; i<this->n_write_ports; i++) {
        Connection* connection = new Connection(this->n_write_ports); // TODO The capacity would be 1. Change this later
        write_port_connections.push_back(connection);
    }
    */
}

SDMemory::~SDMemory() {
    delete write_fifo;
    if(this->tile_loaded) {
        unsigned int num_vns = this->current_tile->get_Num_VNs();
        for(int i=0; i<num_vns; i++) {
            delete VNAT[i];
        }     
        delete[] VNAT;   
    }
    //Deleting the input ports
    for(int i=0; i<this->n_read_ports; i++) {
        delete input_fifos[i];
        delete psum_fifos[i];
    }

}

void SDMemory::setWriteConnections(std::vector<Connection*> write_port_connections) {
    this->write_port_connections=write_port_connections; //Copying all the poiners 
    //assert(this->write_port_connections.size()==this->n_write_ports); 
}

void SDMemory::setReadConnections(std::vector<Connection*> read_connections) {
    assert(read_connections.size() == n_read_ports); //Checking that the number of input ports is valid.
    this->read_connections = read_connections; //Copying all the pointers
}

void SDMemory::setTile(Tile* current_tile) {

    //assert(this->write_port_connections.size()==this->n_write_ports);

    this->iter_S = ceil(dnn_layer->get_S() / (float) current_tile->get_T_S()); //Control the number of S iterations in the MemoryController
    this->iter_K = ceil(dnn_layer->get_K() / (float)current_tile->get_T_K()); //Control the number of K iterations in the MemoryController
    this->iter_X = ceil(dnn_layer->get_X_() / (float) current_tile->get_T_X_()); //Control the number of X iterations in the MemoryController

    unsigned int VNAT_iter_S = 1; //Control the number of S iterations in the VNAT (when writing into the memory). Only neccesary if the memory have to forward psms  
    if(current_tile->get_folding_enabled()) { //If folding is managed by the multipliers (forwarding enabled) then the memory has to be aware and store and send the partial sums to the multipliers
        VNAT_iter_S = this->iter_S;
    }
	
    this->current_tile = current_tile;
    unsigned int num_vn = this->current_tile->get_Num_VNs();
    std::cout << "num vn = " << num_vn << std::endl;
    this->current_output_pixel = 0; //Setting the current pixels computed
    /*要计算的opixels为 iter_N*iter_K*iter_X*iter_Y * num_vn，因为每个神经元都将执行这些迭代。
      请注意，此数字可能与dnn中的opixels不同（它将是 dnn_layer->get_K()*dnn_layer->get_X_()*dnn_layer->get_Y()
      之所以如此，是因为 DNN 执行可能需要多个tile才能完成。*/
    this->output_pixels_to_compute = this->iter_K*this->iter_X*VNAT_iter_S*num_vn;  
    //每个通道的输出总和数。用于避免发送新 k 迭代的包   //Number of output psums per each channel. Used to avoid sending packages of new k iterations if the previous have not been 
   
    this->output_psums_per_channel = this->dnn_layer->get_X_()*VNAT_iter_S; //目前不知道有什么用
    std::cout << "psums to compute " << this->output_pixels_to_compute << std::endl;
    //Assigning to each VN an output initial address depending on N, K, X' and Y'
    this->VNAT = new VNAT_Register*[num_vn];
	std::cout << "(current_vn,addr_offset):" << " ";
    for(unsigned k=0;k<this->current_tile->get_T_K(); k++) {
        for(unsigned x=0; x<this->current_tile->get_T_X_(); x++) {
            unsigned int current_vn =  k*this->current_tile->get_T_X_() + x;
            //The way of calculating the address is different since the data is storaged putting the different k channels consecutive in // memory
            unsigned int addr_offset = x*this->dnn_layer->get_K()*word_size + k*word_size;
            assert(current_vn < num_vn); //Making sure it works
            this->VNAT[current_vn]=new VNAT_Register(current_vn, addr_offset, k, x, iter_S, iter_K, iter_X, this->dnn_layer, this->current_tile);
			std::cout << "(" << current_vn << "," << addr_offset << ")";
        } 
    }
    this->tile_loaded = true;

}

void SDMemory::setLayer(DNNLayer* dnn_layer, address_t input_address, address_t filter_address, address_t output_address, Dataflow dataflow) {
    this->dnn_layer = dnn_layer;
    this->filter_address = filter_address;
    this->input_address = input_address;
    this->output_address = output_address;

    //Dataflow is always ignored here as the order of the loops is always the same when running a CNN
    //
    //i.e., you can change the dataflow in terms of tiling partition but not in terms of loop


    //Updating counters to track the progress

    this->current_S = 0;
    this->current_K = 0;
    this->current_X = 0;

    this->channel_filter_size = dnn_layer->get_R()*dnn_layer->get_S(); //R*S
    this->row_filter_size = dnn_layer->get_S()*dnn_layer->get_C();//S*C
    this->filter_size = dnn_layer->get_R()*dnn_layer->get_S()*dnn_layer->get_C(); //R*S*C
    this->group_size = dnn_layer->get_K()*dnn_layer->get_R()*dnn_layer->get_S()*dnn_layer->get_C(); //filter_size*K
    this->channel_input_size;
    this->row_input_size = dnn_layer->get_G()*dnn_layer->get_C()*dnn_layer->get_Y(); //Row input size = C*Y ,G恒为1
    this->input_size = dnn_layer->get_X()*dnn_layer->get_Y()*dnn_layer->get_C()*dnn_layer->get_G(); // ISize = X*Y*C,（G恒为1）
    this->channel_output_size = dnn_layer->get_X_()*dnn_layer->get_Y_();  //output channel size= X'*Y'
    this->row_output_size =  dnn_layer->get_G()*dnn_layer->get_K()*dnn_layer->get_Y_(); //好像没用到过
    this->output_size = dnn_layer->get_X_()*dnn_layer->get_Y_()*dnn_layer->get_G()*dnn_layer->get_K(); //Osize = X'*Y'*K
    this->output_pixels_to_compute = this->output_size*dnn_layer->get_N();
	std::cout << "X':" << dnn_layer->get_X_() << " " << "Y':" << dnn_layer->get_Y_() << std::endl;
 
}

void SDMemory::cycle() {
    //1.Sending input data over read_connection
    //TODO By the moment we suppose we have enough bandwidth
    std::vector<DataPackage*> data_to_send; //Input and weight temporal storage
    std::vector<DataPackage*> psum_to_send; // psum temporal storage
    this->local_cycle+=1;
    this->sdmemoryStats.total_cycles++; //To track information
    std::cout << "周期" << local_cycle << std::endl;            //新增
	std::cout << "current_S:" << this->current_S << std::endl;
	std::cout << "current_K:" << this->current_K << std::endl;
	std::cout << "current_X:" << this->current_X << std::endl;

    /* CHANGES DONE TO SAVE MEMORY */
    unsigned int current_iteration=current_output_pixel / output_psums_per_channel;
    //unsigned index_G=current_G*this->current_tile->get_T_G(); //恒为零
    unsigned index_K=current_K*this->current_tile->get_T_K();
    unsigned int pck_iteration= index_K;
	std::cout << "current_iteration:" << current_iteration << " ";
	//std::cout << "index_G:" << index_G << " ";
	//std::cout << "index_K:" << index_K << " ";
	std::cout << "pck_iteration:" << pck_iteration << std::endl;
    /* END CHANGES TO SAVE MEMORY */
    if(!this->input_finished && (pck_iteration <= current_iteration)) { //If 
        //1. Weight distribution for the T_K filters
        unsigned window_size = this->current_tile->get_T_R()*this->current_tile->get_T_S()*this->current_tile->get_T_C();//这玩意不就是VN_SIZE吗？
        unsigned folding_shift = 0;
        if(this->current_tile->get_folding_enabled()) { //If there is folding we leave a MS to perform the psum accumulation
           // std::cout << "Aqui hay folding" << std::endl;
            window_size+=1;
            folding_shift+=1;
        }

		/******************分配权重**************************/
        if(!this->weights_distributed) {
			std::cout << "分配权重" << local_cycle << std::endl;
            //For each weight, decide the receivers
			//********************多播*******************//对于相同权重，产生的output为N*X_*Y_个，所以需要将其发送N*X_*Y_次
            if(current_tile->get_T_N()*current_tile->get_T_X_()*current_tile->get_T_Y_() > 1) { //If N*X_*Y_ is greater than 1 then MULTICAST MESSAGE as the same weight is sent multiple times.
                /*for(unsigned g=0; g < current_tile->get_T_G(); g++) {
                    unsigned desp_g = g*current_tile->get_T_K()*current_tile->get_T_X_()*current_tile->get_T_Y_()*window_size;
                    for(unsigned k=0; k < current_tile->get_T_K(); k++) {
                        unsigned desp_k = k*current_tile->get_T_X_()*current_tile->get_T_Y_()*window_size;
                        for(unsigned c=0; c < current_tile->get_T_C(); c++) {
                            for(unsigned r=0; r < current_tile->get_T_R(); r++) {
                                for(unsigned s=0; s < current_tile->get_T_S(); s++) {
                                    //Where do I send this weight
                                    //  To every piece of N
                                    //  To every piece of T_X_
                                    //  To every piece of T_Y_
                                    bool *vector_to_send = new bool[this->num_ms];//用一个位向量标识所有的MS中哪个需要发送权重
                                    //Itiaalizing vector
                                    for(int i=0; i<this->num_ms; i++) {
                                        vector_to_send[i]=false;
                                    }
                                 
                                    for(int n=0; n<this->current_tile->get_T_N(); n++) {
                                        unsigned desp_n = n*this->current_tile->get_T_G()*this->current_tile->get_T_K()*this->current_tile->get_T_X_()*this->current_tile->get_T_Y_()*window_size;
                                        for(int x=0; x<this->current_tile->get_T_X_(); x++) {
                                            for(int y=0; y<this->current_tile->get_T_Y_(); y++) {
                                                unsigned desp_this_neuron = x*this->current_tile->get_T_Y_()*window_size + y*window_size;
                                                vector_to_send[desp_n + desp_g + desp_k + desp_this_neuron +  c*this->current_tile->get_T_S()*this->current_tile->get_T_R() + r*this->current_tile->get_T_S() + s + folding_shift]=true;    //+1 because we skip the first MS again for the folding issue
                                            	//
                                            }
                                        }
                                    }
                                    //Sending the data下面五个用来索引权重的地址
                                    unsigned index_G=current_G*this->current_tile->get_T_G();
                                    unsigned index_K=current_K*this->current_tile->get_T_K();
                                    unsigned index_C=current_C*this->current_tile->get_T_C();
                                    unsigned index_R=current_R*this->current_tile->get_T_R();
                                    unsigned index_S=current_S*this->current_tile->get_T_S();
                                    //this->sdmemoryStats.n_SRAM_weight_reads++; //To track information
				    				data_t data = 0.0;
								    if(((index_R+r) < this->dnn_layer->get_R()) && ((index_S+s) < this->dnn_layer->get_S()) && ((index_C+c) < this->dnn_layer->get_C()) && ((index_K+k) < this->dnn_layer->get_K())) { //Zero-remainder constaint removed
								    	this->sdmemoryStats.n_SRAM_weight_reads++;
				                        data = filter_address[(index_G+g)*this->group_size*word_size + (index_K+k)*this->filter_size*word_size + (index_R+r)*this->row_filter_size*word_size + (index_S+s)*dnn_layer->get_C()*word_size + (index_C+c)];  //Fetching. Note the distribution in memory is interleaving the channels
				                        std::cout << "data权重多播:" << data << std::endl;//新增
								    }

                           
                                    //Creating the package with the weight and the destination vector
                                    DataPackage* pck_to_send = new DataPackage(sizeof(data_t), data, WEIGHT, 0, MULTICAST, vector_to_send, this->num_ms);//data和位向量都有了，那就发送吧
                                    //index_K*this->current_tile->get_T_G() because even though index_K iterations have been calculated previously, there are G groups mapped, so really real index_K*T_G
                                    pck_to_send->setIterationK((index_G)*this->dnn_layer->get_K() + index_K*this->current_tile->get_T_G()); //To avoid sending it to the architecture if the output psums of the previous k channels have not been calculated yet.
                                    this->sendPackageToInputFifos(pck_to_send);

                                }
                            }
                        }
                    } //End K
                } //End G*/
            }

			//***************单播********************//
            else { // N*X_*Y_ == 1 so UNICAST MESSAGES. Each weight is sent once
            	std::cout << "data权重单播:";//新增
                for(unsigned k=0; k < current_tile->get_T_K(); k++) {
                    for(unsigned s=0; s < current_tile->get_T_S(); s++) {
                        //Getting the data from memory
                        unsigned index_K=current_K*this->current_tile->get_T_K();
                        unsigned index_S=current_S*this->current_tile->get_T_S();
                        //this->sdmemoryStats.n_SRAM_weight_reads++; //To track information
                        data_t data = 0.0;
                        if(((index_S+s) < this->dnn_layer->get_S()) && ((index_K+k) < this->dnn_layer->get_K())) { //Zero-remainder constaint removed
                            this->sdmemoryStats.n_SRAM_weight_reads++;
                            data = filter_address[ (index_K+k)*this->filter_size*word_size + (index_S+s)*word_size ];
                            std::cout << data << " ";//新增
                        }
                        
                        //Shift of this weight is g*group_tile_size + k*filter_tile_size + c*filter_channel_tile_size + r*s_tile_size + s
                        unsigned int receiver = k*window_size + s + folding_shift; //+1 because of the ms we leave free for the folding与多播的唯一不同之处，指定唯一的接收者
                        std::cout << "(" << receiver << ")" << " ";//新增
                        //Creating the package with the data in memory and the destination
                        DataPackage* pck_to_send = new DataPackage(sizeof(data_t), data, WEIGHT, 0, UNICAST, receiver);
                        pck_to_send->setIterationK( index_K ); //To avoid sending it to the architecture if the output psums of the previous k channels have not been calculated yet.

                        this->sendPackageToInputFifos(pck_to_send);
                    } //End S
                } //End K
				std::cout << std::endl;//新增
            }

			//
            this->current_S+=1;
            if(this->current_S == iter_S) { //If all the columns completed
                this->current_S = 0;
                this->weights_distributed = true;
            }
        
        } // if(!this->weights_distributed)



		/*************分配输入值********************/
        else { //Input distributions.
        	std::cout << "分配输入值" << local_cycle << std::endl;
            //std::cout << "Sending inputs in the cycle " << this->local_cycle << std::endl;
            //如果是稠密矩阵乘，就是K、M
            unsigned y_inputs = this->current_tile->get_T_S() ; //Number of input rows per batch of the ifmap obtained from the ofmap dimensions. i.e., filter size + number of extra Y_ Neurons mapped times the extra inputs (stride)
            unsigned x_inputs = this->current_tile->get_T_X_(); // Nuumber of input cols per batch of the ifmap obtained from the ofmap dimensions
            //若T_X_,T_Y_为1，则x_inputs、y_inputs就是T_R和T_S；若T_X_,T_Y_不为1，说明一次要产生多个output，滤波器要滑动，一次映射需要的输入值大小就如上式确定

            unsigned output_y_inputs = this->current_tile->get_T_Y_() + (this->dnn_layer->get_S()-1);//这是个啥？如果是稠密矩阵乘，则和上面一样
            unsigned output_x_inputs = this->current_tile->get_T_X_() + (this->dnn_layer->get_R()-1);
            
            //y_init is the first column of the window to send. This might be 0, if all the window must be sent,
            // or the last column if the fw links of the mswitches are enabled in this cycle and therefore bandwidth is saved and just one column must be sent.
            /*y_init 是要发送的窗口的第一列。如果必须发送所有窗口，则此值可能为 0，或者是最后一列如果在此周期中启用了 mswitch 的fw link，因此节省了带宽并且仅须发送一列。*/
            unsigned int y_init = 0; 
			std::cout << "y_inputs: " << y_inputs << " " << "y_init:" << y_init << std::endl;
			std::cout << "x_inputs: " << x_inputs << std::endl;

			//******************根据y_init值选择性发送输入列********************//
            for(unsigned x=0; x<x_inputs; x++) {
                for(unsigned y=y_init; y<y_inputs; y++) { //the number of columns to iterate depends on if the fw links of the MS are enabled多少列用于迭代取决于MS的fw links是否可用？
                    //Each input is replicated k times for this n     每个输入在该批次里面被复制K遍
                    //Creating bool for this element                    
                    bool* destinations = new bool[this->num_ms]; //One destination vector for each n and for each c since we are going to send one value (package) per each batch and channel
                    for(int z=0;z<this->num_ms;z++) {
                        destinations[z]=false;//初始化
                    }
	                    
                   // std::cout << "Buscando VNs para el elemento:  [" << x << "," << y << "]" <<  std::endl;
                   //搜索元素（输入值）对应的VN
				   ///******在可能的VN里面搜索*********///
                    int i=x;//后面还可以优化
                     //   std::cout << "    Trying VN [" << i << ",-]" << std::endl;
                    if((x>=0) && (x < x_inputs))  { // If it's a valid vn //This one, not the next

                        // Y loop (cols)
                        int first_possible_vn_y = (y - (this->current_tile->get_T_S()-1));
                        if(first_possible_vn_y < 0 ) {
                            first_possible_vn_y = 0;
                        }
                        else {
                            first_possible_vn_y = first_possible_vn_y;
                        }
                        int last_possible_vn_y = y;
                        //搜索的第二层循环
                        std::cout << "First possible vn y: " << first_possible_vn_y << " ";
                        std::cout << "Last  possible vn y: " << last_possible_vn_y << std::endl;
                        for(int j=first_possible_vn_y; j<=last_possible_vn_y; j++) {
                            int last_element_vn_j = j + (this->current_tile->get_T_S()-1);
                            int first_element_vn_j = j;
                            // std::cout << "        Trying VN [" << i << "," << j << "]" << std::endl;
                            if((j >= 0) && ((first_element_vn_j+(this->current_tile->get_T_S()-1)) < y_inputs) && (y <= last_element_vn_j) && (y >= first_element_vn_j)) { //If it's a valid vn
                                //Enable this for every k and n
                                int current_vn = i + j; //i times the row size + desp of the currrent row
                                //   std::cout << "Current VN SELECTED: " << current_vn << std::endl;
                                //  std::cout << "            i: " << i << std::endl;
                                // std::cout << "            j: " << j << std::endl;
                                int desp_y_inside_vn = y - j; //Shift y (cols) inside vn
                                int desp_x_inside_vn = x - i; //Shift x (rows) inside vn
                                
                                // std::cout << "            Desp_x_inside_vn: " << desp_x_inside_vn << std::endl;
                                //std::cout << "            Desp_y_inside_vn: " << desp_y_inside_vn << std::endl;
                                int desp_total_inside_vn = desp_x_inside_vn*this->current_tile->get_T_S() + desp_y_inside_vn;
                                    
                                //std::cout << "Desp_total_inside_vn: " << desp_total_inside_vn << std::endl;
                                int current_receiver = current_vn*this->current_tile->get_VN_Size() + desp_total_inside_vn;    //Receiver for an arbitry n and k. //+1 for the folding
                                if(this->current_tile->get_folding_enabled()) {
                                    current_receiver+=1; //1 ms free for psum accumulation
                                }
                                
                                for(int k=0; k<this->current_tile->get_T_K(); k++) {
                                    int desp_k = k*this->current_tile->get_T_X_()*window_size;
                                    //std::cout << "desp_k: " << desp_k << std::endl;
                                    destinations[desp_k + current_receiver]=true; //The package for n will have the desp_n+desp_k ms enabled to receive the activation
                                    //
                                }
                            }
                        }

                    }

                    //Taking the data for each n and send using the corresponding bit_vector in several packages
                    //输入值地址的索引
                    unsigned index_K=current_K*this->current_tile->get_T_K();
                    unsigned index_X=current_X*this->current_tile->get_T_X_();
                    unsigned index_S=current_S*this->current_tile->get_T_S();
                     
					std::cout << "data输入值：";//新增
								//注意destination_vector和destinations的区别，前者才是确定输入值要分配到MS对应的位向量
                                for(int m=0; m<this->num_ms; m++) 
                                    std::cout << destinations[m] << " ";
                                std::cout << std::endl;
                                //this->sdmemoryStats.n_SRAM_input_reads++; 
								data_t data = 0.0;
								if((x < x_inputs) && ((index_S+y) < (y_inputs)) ) { //Zero-remainder constaint removed
					                this->sdmemoryStats.n_SRAM_input_reads++;
                                    //惊呆了，this->dnn_layer->get_Y()竟然为4，等于K?
					                data = input_address[ (index_X + x)*this->dnn_layer->get_Y()*word_size + (y+index_S)*word_size ]; //Read value input(x,y)
					                std::cout << data << " ";//新增
									//读取input_activation
								}
                                //Creating multicast package. Even though the package was unicast, multicast format is used anyway with just one element true in the destination vector
                                DataPackage* pck = new DataPackage(sizeof(data_t), data,IACTIVATION,0, MULTICAST, destinations, this->num_ms);//有修改
                                pck->setIterationK(index_K); //To avoid sending it to the architecture if the output psums of the previous k channels have not been calculated yet.

                                this->sendPackageToInputFifos(pck);//发送数据
 
                    std::cout << std::endl;//新增
                } 
            }

            //TODO iter_X deberia de ser iter_X de inputs, no?         
            //*************Updating variables***********************//
            this->current_S+=1;
            if(this->current_S == this->iter_S) {
                this->current_S = 0;
                 //Updating X
                this->current_X+=1;
                if(this->current_X == this->iter_X) {
                    //If rows finished, updating next N batch
                    this->current_X = 0; //Updating X
                    this->current_K+=1;
                    this->weights_distributed = false; //Distribution enabled for next iteration
                    if(this->current_K==iter_K) {
                        this->current_K=0;     //Update K. Weight distribution neccesary first!                   
                        this->weights_distributed = true; //Avoid distribution
                        this->weights_finished=true;
                        //weights_distributed是一次迭代中分配结束，而weights_finished是所有的分配结束
                        //Current_K updated when weights are distributed
                        //Checking if all the inputs have been delivered ()
                        if(weights_finished) {
                            this->input_finished=true;     //输入值分配结束
                        }
                                       
                    }//end iter_K
                } //end iter_X
            }//end iter_S
        } //End if input_finished
    }
    //Sending the vector
    //std::cout << "Size vector to send: " << data_to_send.size() << std::endl;
    //for(int i=0; i<data_to_send.size(); i++) {
    //    std::cout << "Data package: " << data_to_send[i]->get_data() << " Type: " << data_to_send[i]->get_data_type() << std::endl;
        
   // }
        

    

    //2.Receiving output data from write_connection
    this->receive();
    if(!write_fifo->isEmpty()) {
        //Index the data by using the VN Address Table and the VN id of the packages
        for(int i=0; i<write_fifo->size(); i++) {
            DataPackage* pck_received = write_fifo->pop();
            unsigned int vn = pck_received->get_vn();
            data_t data = pck_received->get_data();
            //std::cout << "Writing data: " << data << std::endl;
            // using the VNAT register to get the address to write
            assert(vn==VNAT[vn]->VN);
            //std::cout << "Memory received a psum " << data << std::endl;
            unsigned int addr_offset = this->VNAT[vn]->addr;
		    if(this->VNAT[vn]->valid_value) { 
	                this->sdmemoryStats.n_SRAM_psum_writes++; //To track information 
	                this->output_address[addr_offset]=data; //ofmap or psum, it does not matter.
		    }

			
            //std::cout << "value written " << data << std::endl;
            current_output_pixel+=1; 
            this->sdmemoryStats.n_SRAM_write_ports_use[pck_received->getOutputPort()]++; //To track information
#ifdef DEBUG_MEM_OUTPUT
            std::cout << "[MEM_OUTPUT] Cycle " <<  local_cycle  << ", STONNE generated a partial output from output port " <<  pck_received->getOutputPort() << " and VN " << pck_received->get_vn() << ". OutputCount ( " << current_output_pixel << "/" << this->output_pixels_to_compute << ")" << std::endl;
#endif
           if((current_output_pixel % 10000) == 0) {
            std::cout << "[MEM_OUTPUT] Cycle " <<  local_cycle  << ", STONNE generated a partial output from output port " <<  pck_received->getOutputPort() << " and VN " << pck_received->get_vn() << ". OutputCount ( " << current_output_pixel << "/" << this->output_pixels_to_compute << ")" << std::endl;
           }

           //std::cout << "Writting in position " << addr_offset << " the value of " << pck_received->get_data() <<  std::endl;

             
            this->VNAT[vn]->current_psum+=1;
            //If forwarding is disabled (current_tile->folding_enabled=false) then this->VNAT[vn]->n_psums will be 1 and therefore, the next condition will neve be true and the psum will never be sent.
            if(this->VNAT[vn]->current_psum < this->VNAT[vn]->n_psums) { //Forward psum data to perform the next psum //
                bool* destination_vector = new bool[this->num_ms];
                for(int i=0; i<this->num_ms; i++) {
                    destination_vector[i]=false;
                }
                destination_vector[vn*this->current_tile->get_VN_Size()]=true;
                DataPackage* pck = new DataPackage(sizeof(data_t), data, PSUM,0, MULTICAST, destination_vector, this->num_ms);
                this->sdmemoryStats.n_SRAM_psum_reads++; //To track information
                this->sendPackageToInputFifos(pck); //Sending the package to fifos
            }
            else {
                this->VNAT[vn]->current_psum=0; //Updating 
            }
           // std::cout << "OUTPUT_PIXELS_TO_COMPUTE: " << output_pixels_to_compute << std::endl;
            //std::cout << "CURRENT_OUTPUT_PIXEL: " << current_output_pixel << std::endl;
            if(current_output_pixel == output_pixels_to_compute) {
                this->execution_finished = true;
            }
            this->VNAT[vn]->update(); //Calculate next output address for that vn
            //Updating address
             
            //std::cout << "Package " << pck_received->get_data() << " received with vn: ";
            //std::cout << vn << std::endl;
           // address_t addr = output_address +
           delete pck_received; //Deleting the current package
            
        }
    }

    //Introducing temporal data to send to the FIFOs used to send 
    this->send(); //Send the content in read_fifo according to the current bw
    std::cout << std::endl;
}

bool SDMemory::isExecutionFinished() {
    return this->execution_finished;
}

/* The traffic generation algorithm generates a package that contains a destination for all the ms. We have to divide it into smaller groups of ms since they are divided into several ports */
void SDMemory::sendPackageToInputFifos(DataPackage* pck) {
    // BROADCAST PACKAGE
    if(pck->isBroadcast()) {
        //Send to all the ports with the flag broadcast enabled
        for(int i=0; i<this->n_read_ports; i++) {
            //Creating a replica of the package to be sent to each port
            DataPackage* pck_new = new DataPackage(pck->get_size_package(), pck->get_data(), pck->get_data_type(), i, BROADCAST); //Size, data, data_type, source (port in this case), BROADCAST
            //Sending the replica to the suitable fifo that correspond with the port
            if(pck->get_data_type() == PSUM) { //Actually a PSUM cannot be broadcast. But we put this for compatibility
                psum_fifos[i]->push(pck_new);
            }          
            else {  //INPUT OR WEIGHT
                //Seting iteration of the package
                pck_new->setIterationK(pck->getIterationK()); //Used to avoid sending packages from a certain iteration without performing the previous.
                input_fifos[i]->push(pck_new);
            }     
           
        }
    }

    // UNICAST PACKAGE
    else if(pck->isUnicast()) {
        //We only have to send the weight to one port and change the destination to adapt it to the subgroup
        unsigned int dest = pck->get_unicast_dest(); //This is according to ALL the mswitches. 
        unsigned int input_port = dest / this->ms_size_per_input_port;
        unsigned int local_dest = dest % this->ms_size_per_input_port;
        //Creating the package 
        DataPackage* pck_new = new DataPackage(pck->get_size_package(), pck->get_data(), pck->get_data_type(), input_port, UNICAST, local_dest); //size, data, type, source (port), UNICAST, dest_local
        //Sending to the fifo corresponding with port input_port
        if(pck->get_data_type() == PSUM) { //Actually a PSUM cannot be broadcast. But we put this for compatibility
            psum_fifos[input_port]->push(pck_new);
        }          
        else {  //INPUT OR WEIGHT
            input_fifos[input_port]->push(pck_new);
            pck_new->setIterationK(pck->getIterationK());
        }

    }

    //MULTICAST PACKAGE 送往其中几个端口
    else { //The package is multicast and then we have to send the package to several ports
        const bool* dest = pck->get_dests();  //One position for mswitch in all the msarray
        bool thereis_receiver;
        for(int i=0; i<this->n_read_ports; i++) { //Checking each port with size this->ms_size_per_input_port each. Total=ms_size
            unsigned int port_index = i*this->ms_size_per_input_port;
            thereis_receiver = false; // To know at the end if the group
            bool* local_dest = new bool[this->ms_size_per_input_port]; //Local destination array for the subtree corresponding with the port i
            for(int j=0; j<this->ms_size_per_input_port; j++) {  //For each ms in the group of the port i
                local_dest[j] = dest[port_index + j]; //Copying the subarray
                if(local_dest[j] == true) {
                    thereis_receiver=true; // To avoid iterating again to know whether the data have to be sent to the port or not.
                }
            }

            if(thereis_receiver) { //If this port have at least one ms to true then we send the data to this port i
                DataPackage* pck_new = new DataPackage(pck->get_size_package(), pck->get_data(), pck->get_data_type(), i, MULTICAST, local_dest, this->ms_size_per_input_port); 
                if(pck->get_data_type() == PSUM) {
                    psum_fifos[i]->push(pck_new);
                }
 
                else {
                    pck_new->setIterationK(pck->getIterationK());
                    input_fifos[i]->push(pck_new);
                    
                }
            }
            else {
                delete[] local_dest; //If this vector is not sent we remove it.
            }
        }
    }

    delete pck; // We have created replicas of the package for the ports needed so we can delete this
} 

void SDMemory::send() {
    //Iterating over each port and if there is data in its fifo we send it. We give priority to the psums
    for(int i=0; i<this->n_read_ports; i++) {
        std::vector<DataPackage*> pck_to_send; 
        if(!this->psum_fifos[i]->isEmpty()) { //If there is something we may send data though the connection
            DataPackage* pck = psum_fifos[i]->pop();
#ifdef DEBUG_MEM_INPUT
            std::cout << "[MEM_INPUT] Cycle " << local_cycle << ", Sending a psum through input port " << i  << std::endl;
#endif
            pck_to_send.push_back(pck);
            this->sdmemoryStats.n_SRAM_read_ports_psums_use[i]++; //To track information
            //Sending to the connection
            this->read_connections[i]->send(pck_to_send);
        }
        //If psums fifo is empty then input fifo is checked. If psum is not empty then else do not compute. Important this ELSE to give priority to the psums and do not send more than 1 pck
        else if(!this->input_fifos[i]->isEmpty()) {
            //If the package belongs to a certain k iteration but the previous k-1 iteration has not finished the package is not sent
            DataPackage* pck = input_fifos[i]->front(); //Front because we are not sure if we have to send it. 
           
            //calculating the current k iteration. i.e., n psums already calculated / number of psums per each channel iteration
            unsigned int current_iteration=current_output_pixel / output_psums_per_channel;
            /*std::cout << "current_output_pixel: " << current_output_pixel << std::endl;
            std::cout << "output_psums_per_channel: " << output_psums_per_channel << std::endl;
            std::cout << "Current_iteration: " << current_iteration << std::endl;
            std::cout << "pck iteration: " << pck->getIterationK() << std::endl; */
            if(pck->getIterationK() <= current_iteration) { //Check if the iteration of the package is older or equal to the current
                if(pck->get_data_type()==WEIGHT) {
                    this->sdmemoryStats.n_SRAM_read_ports_weights_use[i]++; //To track information
#ifdef DEBUG_MEM_INPUT
                    std::cout << "[MEM_INPUT] Cycle " << local_cycle << ", Sending a WEIGHT through input port " << i << std::endl;
#endif
                }  
                else {
                    this->sdmemoryStats.n_SRAM_read_ports_inputs_use[i]++; //To track information
#ifdef DEBUG_MEM_INPUT
                    std::cout << "[MEM_INPUT] Cycle " << local_cycle << ", Sending an INPUT ACTIVATION through input port " << i << std::endl;
#endif
                }
                pck_to_send.push_back(pck); //storing into the vector data type structure used in class Connection 
                this->read_connections[i]->send(pck_to_send); //Sending the input or weight through the connection
                input_fifos[i]->pop(); //pulling from fifo
            }
            else {
#ifdef DEBUG_MEM_INPUT
    std::cout << "[MEM_INPUT] Cycle " << local_cycle << ", input port " << i << " waiting for iteration " << current_iteration << std::endl;
#endif
            }            

        }
            
    }
}

//TODO Remove this connection
void SDMemory::receive() { //TODO control if there is no space in queue
    if(this->write_connection->existPendingData()) {
        std::vector<DataPackage*> data_received = write_connection->receive();
        for(int i=0; i<data_received.size(); i++) {
            write_fifo->push(data_received[i]);
        }
    }
    for(int i=0; i<write_port_connections.size(); i++) { //For every write port
        if(write_port_connections[i]->existPendingData()) {
            std::vector<DataPackage*> data_received = write_port_connections[i]->receive();
             for(int i=0; i<data_received.size(); i++) {
                 write_fifo->push(data_received[i]);
             }
        }    
    }
}

void SDMemory::printStats(std::ofstream& out, unsigned int indent) {
    out << ind(indent) << "\"SDMemoryStats\" : {" << std::endl; //TODO put ID
    this->sdmemoryStats.print(out, indent+IND_SIZE);
    out << ind(indent) << "}"; //Take care. Do not print endl here. This is parent responsability
}

void SDMemory::printEnergy(std::ofstream& out, unsigned int indent) {
    /*
        This component prints:
            - The number of SRAM reads
            - The number of SRAM writes

        Note that the number of times that each port is used is not shown. This is so because the use of those wires are
        taken into account in the CollectionBus and in the DSNetworkTop
   */

   counter_t reads = this->sdmemoryStats.n_SRAM_weight_reads + this->sdmemoryStats.n_SRAM_input_reads + this->sdmemoryStats.n_SRAM_psum_reads;
   counter_t writes = this->sdmemoryStats.n_SRAM_psum_writes;
   out << ind(indent) << "GLOBALBUFFER READ=" << reads; //Same line
   out << ind(indent) << " WRITE=" << writes << std::endl;
        
}

