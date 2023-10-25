
#include "SparseSDMemory.h"
#include <assert.h>
#include <iostream>
#include <algorithm>
#include "utility.h"

SparseSDMemory::SparseSDMemory(id_t id, std::string name, Config stonne_cfg, Connection* write_connection) : MemoryController(id, name) {
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
    this->write_fifo = new Fifo(write_buffer_capacity);
    for(int i=0; i<this->n_read_ports; i++) {
        Fifo* read_fi = new Fifo(this->write_buffer_capacity);
        Fifo* psum_fi = new Fifo(this->write_buffer_capacity);        input_fifos.push_back(read_fi);//每一个读端口创建一个read_fi，加入input_fifos;
        psum_fifos.push_back(psum_fi);//每一个读端口创建一个psum_fi加入psum_fifos
        this->sdmemoryStats.n_SRAM_read_ports_weights_use.push_back(0);  //To track information
        this->sdmemoryStats.n_SRAM_read_ports_inputs_use.push_back(0);   //To track information
        this->sdmemoryStats.n_SRAM_read_ports_psums_use.push_back(0);    //To track information
    }
    for(int i=0; i<this->n_write_ports; i++) {  //To track information
        this->sdmemoryStats.n_SRAM_write_ports_use.push_back(0);  //To track information
    }  //To track information
    this->configuration_done = false;
    this->stationary_distributed = false;
    this->stationary_finished = false;
    this->stream_finished = false;
    this->execution_finished = false;
    this->metadata_loaded = false;
    this->layer_loaded = false;
    this->local_cycle=0;
    this->str_current_index = 0;
    this->current_state = CONFIGURING;
    this->sta_counters_table = NULL;
    this->str_counters_table = NULL;
    this->current_output = 0;
    this->output_size = 0;
    this->sta_current_index_metadata = 0; //Stationary matrix current index (e.g., row in MK)
    this->sta_current_index_matrix = 0;
    this->sta_iter_completed = false;
    this->current_output_iteration = 0;
    this->output_size_iteration = 0;
    this->sta_current_j_metadata = 0;
    this->sta_last_j_metadata = 0;
    this->n_ones_sta_matrix=0;
    this->n_ones_str_matrix=0;

	//新增
	for(int i=0;i<this->K;i++){
		Fifo* needB_fifo =new Fifo(this->N);
		needB_fifos.push_back(needB_fifo);
	}
}

SparseSDMemory::~SparseSDMemory() {
    delete write_fifo;
    //Deleting the input ports
    for(int i=0; i<this->n_read_ports; i++) {
        delete input_fifos[i];
        delete psum_fifos[i];
    }
    
    if(this->layer_loaded) {
        delete[] sta_counters_table;
	delete[] str_counters_table;
    }

	for(int i=0;i<this->K;i++){//新增
		delete needB_fifos[i];
	}
}

void SparseSDMemory::setWriteConnections(std::vector<Connection*> write_port_connections) {
    this->write_port_connections=write_port_connections; //Copying all the poiners 
    //assert(this->write_port_connections.size()==this->n_write_ports); 
}

void SparseSDMemory::setReadConnections(std::vector<Connection*> read_connections) {
    assert(read_connections.size() == n_read_ports); //Checking that the number of input ports is valid.
    this->read_connections = read_connections; //Copying all the pointers
}

void SparseSDMemory::setLayer(DNNLayer* dnn_layer, address_t MK_address, address_t KN_address, address_t output_address, Dataflow dataflow) {//MK_address其实就是MK_matrix，都为float* 类型
    this->dnn_layer = dnn_layer;
    assert(this->dnn_layer->get_layer_type()==GEMM);  // This controller only supports GEMM with sparsity
    this->dataflow = dataflow; 

    this->output_address = output_address;
    this->layer_loaded = true;


    //Loading parameters according to the equivalence between CNN layer and GEMM. This is done
    //in this way to keep the same interface.
    this->M = this->dnn_layer->get_K();
    this->K = this->dnn_layer->get_S();   //Be careful. K in GEMMs (SIGMA taxonomy) is not the same as K in CNN taxonomy (number of filters)
    this->N = this->dnn_layer->get_X();  //In this case both parameters match each other.
    std::cout << "Value of M=" << M << std::endl;
    std::cout << "Value of N=" << N << std::endl;
    std::cout << "Value of K=" << K << std::endl;
    sdmemoryStats.dataflow=dataflow; 

    if(dataflow==MK_STA_KN_STR) {
		std::cout << "Running MK_STA_KN_STR Dataflow" << std::endl;
        this->STA_address = MK_address;
		this->dim_sta = M;
		this->STR_address = KN_address;
		this->dim_str = N;

    
	//MK_sta_ KN STR dataflow. According to the distribution of the bitmap
        this->STA_DIST_ELEM=1;
        this->STA_DIST_VECTOR=K;

        this->STR_DIST_ELEM=dim_str;//N
        this->STR_DIST_VECTOR=1;

		this->OUT_DIST_VN=dim_str;//N
        this->OUT_DIST_VN_ITERATION=1;
	 

    }

    else if(dataflow==MK_STR_KN_STA) {
		std::cout << "Running MK_STR_KN_STA Dataflow" << std::endl;
        this->STA_address = KN_address;
		this->dim_sta = N;
		this->STR_address = MK_address;
		this->dim_str= M;

		this->STA_DIST_ELEM=dim_sta;
		this->STA_DIST_VECTOR=1;

		this->STR_DIST_ELEM=1;
		this->STR_DIST_VECTOR=K;

	 	this->OUT_DIST_VN=1;
        this->OUT_DIST_VN_ITERATION=dim_sta;



    }

    else {
        std::cout << "Dataflow not recognised" << std::endl;
	assert(false);
    }


    this->output_size = dim_sta*dim_str;
    this->sta_counters_table = new unsigned int[dim_sta*K];
    this->str_counters_table = new unsigned int[dim_str*K];
	//新增初始化
	for(int i=0;i<dim_sta*K;i++)
		sta_counters_table[i]=9;
	for(int i=0;i<dim_str*K;i++)
		str_counters_table[i]=9;
}


void SparseSDMemory::setSparseMetadata(metadata_address_t MK_metadata, metadata_address_t KN_metadata, metadata_address_t output_metadata) {
    if(this->dataflow==MK_STA_KN_STR) {
        this->STA_metadata = MK_metadata;
        this->STR_metadata = KN_metadata;
        this->output_metadata = output_metadata;
    }

    else if(dataflow==MK_STR_KN_STA) {
        this->STA_metadata = KN_metadata;
		this->STR_metadata = MK_metadata;
		this->output_metadata = output_metadata;
    }

    else {
        std::cout << "Dataflow not recognised" << std::endl;
        assert(false);
    }

    this->metadata_loaded = true;
}

//Load bitmaps and CSR
void SparseSDMemory::setSparseMetadata(metadata_address_t MK_metadata, metadata_address_t KN_metadata, metadata_address_t output_metadata,metadata_address_t MK_rowptr,metadata_address_t MK_rowindex,metadata_address_t MK_colindex,metadata_address_t KN_rowptr,metadata_address_t KN_rowindex,metadata_address_t KN_colindex,metadata_address_t output_rowptr,metadata_address_t output_rowindex,metadata_address_t output_colindex) {
    if(this->dataflow==MK_STA_KN_STR) {
        this->STA_metadata = MK_metadata;
        this->STR_metadata = KN_metadata;
        this->output_metadata = output_metadata;
		/**********新增load_CSR***************/
		this->STA_rowptr = MK_rowptr;
		this->STA_rowindex = MK_rowindex;
		this->STA_colindex = MK_colindex;
		this->STR_rowptr = KN_rowptr;
		this->STR_rowindex = KN_rowindex;
		this->STR_colindex = KN_colindex;
		/**********新增***************/
    }

    else if(dataflow==MK_STR_KN_STA) {
        this->STA_metadata = KN_metadata;
		this->STR_metadata = MK_metadata;
		this->output_metadata = output_metadata;
		/**********新增load_CSR***************/
		this->STA_rowptr = KN_rowptr;
		this->STA_rowindex = KN_rowindex;
		this->STA_colindex = KN_colindex;
		this->STR_rowptr = MK_rowptr;
		this->STR_rowindex = MK_rowindex;
		this->STR_colindex = MK_colindex;
		/**********新增***************/
    }

    else {
        std::cout << "Dataflow not recognised" << std::endl;
        assert(false);
    }

    this->metadata_loaded = true;

}



void SparseSDMemory::cycle() {
    //Sending input data over read_connection
    assert(this->layer_loaded);  // Layer has been loaded
    assert(this->metadata_loaded); //Metadata for sparsity has been loaded
    std::vector<DataPackage*> data_to_send; //Input and weight temporal storage
    std::vector<DataPackage*> psum_to_send; // psum temporal storage
    this->local_cycle+=1;
    this->sdmemoryStats.total_cycles++; //To track information

 //*******************CONFIGURING***********************//
    if(current_state==CONFIGURING) {   //If the architecture has not been configured
    	std::cout << "/***CONFIGURING!!!***/" << local_cycle << std::endl;//新增
        int i=sta_current_index_metadata;  //Rows  sta_current_index_metadata初始为0，后面状态转变的时候会改变
		int j=0;  //Columns
		int n_ms = 0; //Number of multipliers assigned
		int n_current_cluster = 0;
		this->configurationVNs.clear();
		this->vnat_table.clear();
		/******计算STA矩阵的DEST表？*******/
		if(this->sta_current_j_metadata > 0)  { // We are managing one cluster with folding  构造时sta_current_j_metadata=0，
			std::cout<<"打印1" << std::endl;
            n_ms++; //One for the psum
		    j=this->sta_current_j_metadata;
		    while((n_ms < this->num_ms) && (j < K)) { //TODO change MK if it is another dw
		    		//i*K + j
	                if(this->STA_metadata[i*STA_DIST_VECTOR + j*STA_DIST_ELEM]) { //If the bit is enabled in the stationary bitmap
	                    //Add to the cluster
	                    this->sta_counters_table[i*STA_DIST_VECTOR + j*STA_DIST_ELEM]=n_ms; //DEST
	                    n_ms++;
	                    n_current_cluster++;
	                }
	                
				j++;

		    }
		    /*
		    //Making sure that if there is next cluster, that cluster have size >=3
		    if((j < K) && ((K-j) < 3)) {
	                int n_elements_to_make_cluster_3 = 3-(K-j);
	                j-=n_elements_to_make_cluster_3;
	                n_current_cluster-=n_elements_to_make_cluster_3;
		    } */

            SparseVN VN(n_current_cluster, true); //Here folding is enabled and the SparseVN increments 1 to size
            this->configurationVNs.push_back(VN);
	    	this->vnat_table.push_back(0); //Adding the current calculation (row or column) of this VN.
	    	//Searching if there are more values in the next cluster. If not, update sta_last_j_metadata to K to indicate that in the next iteration the next sta dim must be evaluated
	    	int remaining_values = 0;
	    	for(int r=j; r<K; r++) {
                if(this->STA_metadata[i*STA_DIST_VECTOR + r*STA_DIST_ELEM]) {
                    remaining_values+=1;
	        	}
			}
		    if(remaining_values > 0) {
		        this->sta_last_j_metadata=j;
		    }

		    else {
	                this->sta_last_j_metadata = K;
		    }
		}

		else { //Whole rows
			//1.每次执行完该while循环，要把num_ms个乘法器分配完(尽可能多的映射更多的行)，如果有一行映射不完就算了，下一次CONFIGURING阶段从这行开始重新映射
			std::cout<<"打印2" << std::endl;
		    while((n_ms < this->num_ms) && (i*K+j < dim_sta*K)) { //TODO change MK if it is another dw
	            if(this->STA_metadata[i*STA_DIST_VECTOR + j*STA_DIST_ELEM]) { //If the bit is enabled in the stationary bitmap
	            //Add to the cluster
	            //i*K+j
				    this->sta_counters_table[i*STA_DIST_VECTOR + j*STA_DIST_ELEM]=n_ms; //DEST
				    std::cout << "STA_DEST (" << i << ", " << j << "): " << n_ms << std::endl;
				    n_ms++;
				    n_current_cluster++; 
		        }

	            j++; // Next elem in vector
	            
		        if(j==K) {//每一行创建一个VN（只要这行不全为0）
			    	//Change cluster since we change of vector
	                j=0; //elem = 0
			    	i++; // Next vector
				    if(n_current_cluster > 0) {
		                //Creating the cluster for this row
				        SparseVN VN(n_current_cluster, false);//false指的是不支持折叠
		                this->configurationVNs.push_back(VN); //Adding to the list 
				        this->vnat_table.push_back(0); //Adding the current calculation (row or column) of this VN.
				        n_current_cluster = 0;
				    }
		        }

		    }
			
			//2.比如1203，4000是STA矩阵的两行，第一次分配完1234，构造VN（123），然后发现4后面全为0，没有remaining_values，于是构造VN（4）
		    if((this->configurationVNs.size() > 0) && (j<K)) {
	            //Find if there is a last cluster
				int remaining_values = 0;
	            for(int r=j; r<K; r++) {//i*K+r
	            	if(this->STA_metadata[i*STA_DIST_VECTOR + r*STA_DIST_ELEM]) {
	                        remaining_values+=1;
	                }
	            }
				//Its the last element
	            if(remaining_values == 0) {
			    	if(n_current_cluster > 0) {
						SparseVN VN(n_current_cluster, false);
	                    this->configurationVNs.push_back(VN); //Adding to the list
	                    this->vnat_table.push_back(0); //Adding the current calculation (row or column) of this VN.
			    	}
	            }
		    }
			
			//3.两种情况：
				//a.最后STA矩阵分配完了，没把ms分配满，不满就不满吧，最后剩的这些作一个VN 		
				//b.ms分配完了但没换行，这时候不是得折叠吗？所以就涉及到了sta_last_j_metadata，sta_current_j_metadata这两个变量，在状态转变部分的代码也会用到
	    	if(this->configurationVNs.size()==0) { //If any entire cluster fits, then folding is needed to manage this cluster
		  
				/*if((K-j) < 3) { //The next cluster must have cluster size greater or equal than 3
		                    int n_elements_to_make_cluster_3 = 3-(K-j);
				    j-=n_elements_to_make_cluster_3;
				    n_current_cluster-=n_elements_to_make_cluster_3;
				}*/
			
		        SparseVN VN(n_current_cluster, false); //Here folding is still disabled as this is the first iteration
		        this->configurationVNs.push_back(VN);
				this->vnat_table.push_back(0); //Adding the current calculation (row or column) of this VN.
			    //Searching if there are more values in the next cluster. If not, update sta_last_j_metadata to K to indicate that in the next iteration the next sta dim must be evaluated
	            int remaining_values = 0;
	            for(int r=j; r<K; r++) {
	                if(this->STA_metadata[i*STA_DIST_VECTOR + r*STA_DIST_ELEM]) {
	                        remaining_values+=1;
	                }
	            }
	            if(remaining_values > 0) {
	               this->sta_last_j_metadata=j;
	            }

	            else {
	            	this->sta_last_j_metadata = K;
	            }

              
	    	}

			//4.
	   		else { //If there is at least one cluster, then all of them has size K and it is necessary to stream K
			   //K elements
			   this->sta_last_j_metadata=this->K;
            }

        } //end else whole rows

		/******计算STR矩阵的SRC表*******/
		//Calculating the STR SOURCE TABLE with the indexes of each value
		unsigned int source_id = 0;
		for(int i=0; i<dim_str; i++) {
	            for(int j=0; j<K; j++) {
	                if(this->STR_metadata[i*STR_DIST_VECTOR+j*STR_DIST_ELEM]) {
			    		std::cout << "STR_SRC (" << i << ", " << j << "): " << source_id << std::endl;
	                    this->str_counters_table[i*STR_DIST_VECTOR + j*STR_DIST_ELEM]=source_id;
			    		source_id++;
					}
		    	}
		}

		this->n_ones_str_matrix = source_id;


		//Once the VNs has been selected, lets configure the RN and MN
	    // Configuring the multiplier network
		if(this->configurationVNs.size()==0) {//簇大小超过了每行的ms数？
	            std::cout << "Cluster size exceeds the number of multipliers in row " << this->sta_current_index_metadata << std::endl;
		    assert(false);
		}
		for(int i=0; i<this->configurationVNs.size(); i++) {//打印所有VN的大小,但每次配置阶段开始都会configurationVNs.clear(),故只有一个VN
	            std::cout << "Found a VN of size " << this->configurationVNs[i].get_VN_Size() << std::endl;
	    }
	    this->sdmemoryStats.n_sta_vectors_at_once_avg+=this->configurationVNs.size(); //accumul
		if(this->configurationVNs.size() > this->sdmemoryStats.n_sta_vectors_at_once_max) {
	        this->sdmemoryStats.n_sta_vectors_at_once_max = this->configurationVNs.size();
		}
		this->sdmemoryStats.n_reconfigurations++;

		//每次CONFIGURING阶段都会编译一次
		//这里会编译MN和RN（configureSparseSignals其实就是根据分配好的VN来确定adder的config_mode?）
		std::cout << "Configuring the Networks" << std::endl;
		this->multiplier_network->resetSignals(); //Reseting the values to default
		this->multiplier_network->configureSparseSignals(this->configurationVNs, this->dnn_layer, this->num_ms);//////////////////////配置MN
		//Configuring the reduce network
		this->reduce_network->resetSignals(); //Reseting the values to default
		this->reduce_network->configureSparseSignals(this->configurationVNs, this->dnn_layer, this->num_ms);//////////////////////配置RN
		
		std::cout << "End configuring" << std::endl;
		//Number of psums to calculate in this iteration
		this->output_size_iteration=this->configurationVNs.size()*this->dim_str;
		
		
		std::cout << "sta_counters_table:" << std::endl;//新增：打印sta的DEST表
		for(int i=0;i<dim_sta;i++){
			for(int j=0;j<K;j++)
				std::cout << sta_counters_table[i*K+j] << " ";
			std::cout << std::endl;
		}
		std::cout << "str_counters_table:" << std::endl;//新增：打印str的src表
		for(int i=0;i<K;i++){
			for(int j=0;j<dim_str;j++)
				std::cout << str_counters_table[i*dim_str+j] << " ";
			std::cout << std::endl;
		}
		std::cout << "/***END OF CONFIGURING!!!***/" << local_cycle << std::endl;//新增
    }


//*******************DIST_STA_MATRIX***********************//
    else if(current_state == DIST_STA_MATRIX) {
	std::cout << "/***DIST_STA_MATRIX!!!***/" << local_cycle << std::endl;//新增
		needB.clear();
       //Distribution of the stationary matrix
       unsigned int dest = 0; //MS destination
       unsigned int sub_address = 0;
    	std:: cout << "分配STA矩阵：";
       for(int i=0; i<this->configurationVNs.size(); i++) {//有i个VN
	   	   int j=0;
	  	   if(this->configurationVNs[i].getFolding()) {
              j=1;
	       	  dest++; //Avoid the one in charge of the psum
	       }
           for(; j<this->configurationVNs[i].get_VN_Size(); j++) {
	          //Accessing to memory
	          data_t data = this->STA_address[sta_current_index_matrix+sub_address]; //In both dataflows adjacents elements are consecutive in mem
	          sdmemoryStats.n_SRAM_weight_reads++;
	          this->n_ones_sta_matrix++; 
			  /********/
			  unsigned int row_index = this->STA_rowindex[sta_current_index_matrix+sub_address];
			  unsigned int col_index = this->STA_colindex[sta_current_index_matrix+sub_address];
			  needB.push_back(col_index);
			  /********/
	          DataPackage* pck_to_send = new DataPackage(sizeof(data_t), data, WEIGHT, 0, UNICAST, dest,row_index,col_index);
	          this->sendPackageToInputFifos(pck_to_send);
              dest++;
	          sub_address++;
			  std::cout << data << " ";
	       }
		   std::cout << std::endl;
       }
	   std::cout << std::endl;
	  //新增（后面都是） 
	   sort(needB.begin(),needB.end());
	   needB.erase(unique(needB.begin(), needB.end()), needB.end());//删除needB中的重复元素
	   std::cout << "needB:";
	   for(int i=0;i<needB.size();i++)
	   		std::cout << needB[i] << " ";
	   std::cout << std::endl;
	   for(int i=0;i<needB.size();i++){
	   		unsigned int B_rowindex = needB[i];
			std::cout << "B" << B_rowindex << "行：";
	   		for(unsigned int j=this->STR_rowptr[B_rowindex];j<this->STR_rowptr[B_rowindex+1];j++){

			
			   bool* destinations = new bool[this->num_ms];
	  		   for(int k=0; k<this->num_ms; k++) {
	  	             destinations[k]=false;
	  		   }

			   //Send the value i times with bit on bitmap enabled
			   //Searching for destinations for this elemeent
			   unsigned int first_sta_vector = sta_current_index_metadata;//sta已分配到的当前行
			   unsigned int last_sta_vector = sta_current_index_metadata + this->configurationVNs.size();
		       for(int k=first_sta_vector; k<last_sta_vector; k++) { //For each vector in the sta matrix could exist a value to send the str value
		           if(STA_metadata[k*STA_DIST_VECTOR + B_rowindex*STA_DIST_ELEM]) { //ADD Destination
				   		unsigned int dest = sta_counters_table[k*STA_DIST_VECTOR + B_rowindex*STA_DIST_ELEM];
		                destinations[dest]=true;
			       }
		   	   }
/*
			   DataPackage* pck = new DataPackage(sizeof(data_t), STR_address[j], IACTIVATION, 0);
				//DataPackage* pck = new DataPackage(sizeof(data_t), STR_address[j], IACTIVATION, 0, MULTICAST, destinations, this->num_ms );
				needB_fifos[B_rowindex]->push(pck);*/
				std::cout << STR_address[j] << " ";
	   		}
	   
			std::cout << std::endl;
	   }
		std::cout << "/***END OF DIST_STA_MATRIX!!!***/" << local_cycle << std::endl;//新增
    }


//*******************DIST_STR_MATRIX***********************//
    else if(current_state == DIST_STR_MATRIX) {
		std::cout << "/***DIST_STR_MATRIX!!!***/" << local_cycle << std::endl;//新增
       int init_point_str = this->sta_current_j_metadata; std::cout << "sta_current_j_metadata"<<sta_current_j_metadata<<std::endl;
       int end_point_str = this->sta_last_j_metadata;std::cout << "sta_last_j_metadata"<<sta_last_j_metadata<<std::endl;
       if(this->sta_current_j_metadata > 0) { //If folding is enabled there is just a row on  fly
           assert(this->configurationVNs.size()==1);
           //send psum
		   unsigned int addr_offset = (sta_current_index_metadata)*OUT_DIST_VN + str_current_index*OUT_DIST_VN_ITERATION;
		   bool* destinations = new bool[this->num_ms];
	       for(int i=0; i<this->num_ms; i++) {
	           destinations[i]=false;
	       }
		   destinations[0]=true;

		   data_t psum = this->output_address[addr_offset];  //Reading the current psum
		   DataPackage* pck = new DataPackage(sizeof(data_t), psum, PSUM,0, MULTICAST, destinations, this->num_ms);
	           this->sdmemoryStats.n_SRAM_psum_reads++; //To track information
		   this->sendPackageToInputFifos(pck);
	   
       }
	   
	   std::cout << "分配STR矩阵：";//新增
	   //不折叠的情况下这里应该是0到4，一次发送一列str矩阵
	   
       for(int j=init_point_str; j<end_point_str; j++) {   //For each element in the current vector in the str matrix
           //Creating the bit vector for this value
		   bool* destinations = new bool[this->num_ms];
  		   for(int i=0; i<this->num_ms; i++) {
  	             destinations[i]=false;
  		   }

		   //Send the value i times with bit on bitmap enabled
		   //Searching for destinations for this elemeent
		   unsigned int first_sta_vector = sta_current_index_metadata;//sta已分配到的当前行
		   unsigned int last_sta_vector = sta_current_index_metadata + this->configurationVNs.size();
	       for(int i=first_sta_vector; i<last_sta_vector; i++) { //For each vector in the sta matrix could exist a value to send the str value
	           if(STA_metadata[i*STA_DIST_VECTOR + j*STA_DIST_ELEM]) { //ADD Destination
			   		unsigned int dest = sta_counters_table[i*STA_DIST_VECTOR + j*STA_DIST_ELEM];
	                destinations[dest]=true;
		       }
		   }

		   //Accessing to the element in the str matrix
		   data_t data;
		   if(STR_metadata[str_current_index*STR_DIST_VECTOR + j*STR_DIST_ELEM]) { 
		       unsigned int src = str_counters_table[str_current_index*STR_DIST_VECTOR + j*STR_DIST_ELEM];
		       data = STR_address[src];
		       sdmemoryStats.n_SRAM_input_reads++;
			   std::cout << data << " ";//新增
		   }

		   else {
	             data=0.0; //If the STA matrix has a value then the STR matrix must be sent even if the value is 0
	             std::cout << data << " ";//新增
	       }

		   //Creating the package
	       DataPackage* pck = new DataPackage(sizeof(data_t), data,IACTIVATION,0, MULTICAST, destinations, this->num_ms);

		   this->sendPackageToInputFifos(pck);//此函数会筛选一遍，当STA矩阵一行的第k个元素为0时，STR矩阵每列的第k个元素就不会被压入FIFO里面
	   } 
	   std::cout << std::endl;//新增

	       str_current_index++;
	   std::cout << "/***END OF DIST_STR_MATRIX!!!***/" << local_cycle << std::endl;//新增
    }


         
//*******************开始接收数据***********************//
    //Receiving output data from write_connection
    this->receive();
    if(!write_fifo->isEmpty()) {
        //Index the data by using the VN Address Table and the VN id of the packages
        for(int i=0; i<write_fifo->size(); i++) {
            DataPackage* pck_received = write_fifo->pop();
            unsigned int vn = pck_received->get_vn();
            data_t data = pck_received->get_data();
            this->sdmemoryStats.n_SRAM_psum_writes++; //To track information 
            
            //vnat_table在这里的作用：算该输出结果在输出矩阵中对应的索引，然后自增是？
	        unsigned int addr_offset = (sta_current_index_metadata+vn)*OUT_DIST_VN + vnat_table[vn]*OUT_DIST_VN_ITERATION; 
	        vnat_table[vn]++; 
			
            this->output_address[addr_offset]=data; //ofmap or psum, it does not matter.
            current_output++;
	        current_output_iteration++;
			
	        if(current_output_iteration==output_size_iteration) {
                current_output_iteration = 0;
		        sta_iter_completed=true;
	        }
            delete pck_received; //Deleting the current package
            
        }
    }


//*******************状态转变***********************//
    //Transitions
    if(current_state==CONFIGURING) {
        current_state=DIST_STA_MATRIX;
    }

    else if(current_state==DIST_STA_MATRIX) {
        current_state=DIST_STR_MATRIX;
    }

    else if(current_state==DIST_STR_MATRIX  && str_current_index==dim_str) {
		current_state = WAITING_FOR_NEXT_STA_ITER;
		std::cout << "/***WAITING_FOR_NEXT_STA_ITER!!!***/" << local_cycle+1 << std::endl;//新增
    }

    else if(current_state==WAITING_FOR_NEXT_STA_ITER && sta_iter_completed) {
    
		this->str_current_index = 0;
		this->sta_iter_completed=false;
        if(this->configurationVNs.size()==1) {//If there is only one VN, then maybe foliding has been needed
            this->sta_current_j_metadata=this->sta_last_j_metadata;
	   // if(this->configurationVNs[0].getFolding()) {
           //     this->sta_current_j_metadata-=1;
	   // }

	    	if(this->sta_current_j_metadata == this->K) { //If this is the end of the cluster, it might start to the next 
                this->sta_current_index_metadata+=1;
				this->sta_current_j_metadata = 0;
				std::cout << "STONNE: STA dimensions completed (" << this->sta_current_index_metadata << "/" << this->dim_sta << ")" << std::endl;
            }
		}

		else {
	    	this->sta_current_index_metadata+=this->configurationVNs.size();
	    	std::cout << "STONNE: STA dimensions completed (" << this->sta_current_index_metadata << "/" << this->dim_sta << ")" << std::endl;
	    	this->sta_current_j_metadata = 0;
		}
		unsigned int total_size = 0;
		for(int i=0; i<this->configurationVNs.size(); i++) {
            total_size+=this->configurationVNs[i].get_VN_Size();
	    	if(this->configurationVNs[i].getFolding()) {
                total_size-=1; //Sustract the -1 of the extra multiplier 
	    	}
        }
		this->sta_current_index_matrix+=total_size;//

		if(sta_current_index_metadata>=this->dim_sta) {
		    //Calculating sparsity values  and some final stats
		    unsigned int sta_metadata_size = this->dim_sta*K;
		    unsigned int str_metadata_size = this->dim_str*K;
		    unsigned int sta_zeros = sta_metadata_size - this->n_ones_sta_matrix;
		    unsigned int str_zeros = str_metadata_size - this->n_ones_str_matrix;
	            sdmemoryStats.sta_sparsity=(counter_t)((100*sta_zeros) / sta_metadata_size);
		    sdmemoryStats.str_sparsity=(counter_t)((100*str_zeros) / str_metadata_size);
		    this->sdmemoryStats.n_sta_vectors_at_once_avg = this->sdmemoryStats.n_sta_vectors_at_once_avg / this->sdmemoryStats.n_reconfigurations;
	            this->execution_finished = true; //if the last sta cluster has already be calculated then finish the sim
		    current_state = ALL_DATA_SENT;
			std::cout<< "ALL_DATA_SENT!!!" << local_cycle+1 << std::endl;//新增
		}

		else { 
	         	current_state=CONFIGURING;
		}
    }

   

//*******************发送数据***********************//
    this->send();
	std::cout << std::endl;
}

bool SparseSDMemory::isExecutionFinished() {
    return this->execution_finished;
}

/* The traffic generation algorithm generates a package that contains a destination for all the ms. We have to divide it into smaller groups of ms since they are divided into several ports */
void SparseSDMemory::sendPackageToInputFifos(DataPackage* pck) {
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
			//std::cout << "psum_fifo" << input_port << "push " <<pck_new->get_data() << std::endl;//新增
        }          
        else {  //INPUT OR WEIGHT
            input_fifos[input_port]->push(pck_new);
			//std::cout << "input_fifo" << input_port << "push " <<pck_new->get_data() << std::endl;//新增
            pck_new->setIterationK(pck->getIterationK());
        }

    }

    //MULTICAST PACKAGE 
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
					//std::cout << "psum_fifo" << i << "push " <<pck_new->get_data() << std::endl;//新增
                }
 
                else {
                    pck_new->setIterationK(pck->getIterationK());
                    input_fifos[i]->push(pck_new);
                    //std::cout << "input_fifo" << i << "push " <<pck_new->get_data() << std::endl;//新增
                }
            }
            else {
                delete[] local_dest; //If this vector is not sent we remove it.
            }
        }
    }

    delete pck; // We have created replicas of the package for the ports needed so we can delete this
} 

void SparseSDMemory::send() {
    //Iterating over each port and if there is data in its fifo we send it. We give priority to the psums
	std::cout << "send!" << local_cycle << std::endl;
    for(int i=0; i<this->n_read_ports; i++) {//
        std::vector<DataPackage*> pck_to_send; 

        if(!this->psum_fifos[i]->isEmpty()) { //If there is something we may send data though the connection
            DataPackage* pck = psum_fifos[i]->pop();
#ifdef DEBUG_MEM_INPUT
            std::cout << "[MEM_INPUT] Cycle " << local_cycle << ", Sending a psum through input port " << i  << std::endl;
#endif
			std::cout << "psum:" << pck->get_data() << std::endl;
            pck_to_send.push_back(pck);
            this->sdmemoryStats.n_SRAM_read_ports_psums_use[i]++; //To track information
            //Sending to the connection
            this->read_connections[i]->send(pck_to_send);
        }
		//啥意思？
        //If psums fifo is empty then input fifo is checked. If psum is not empty then else do not compute. Important this ELSE to give priority to the psums and do not send more than 1 pck
        else if(!this->input_fifos[i]->isEmpty()) {
            //If the package belongs to a certain k iteration but the previous k-1 iteration has not finished the package is not sent
            DataPackage* pck = input_fifos[i]->front(); //Front because we are not sure if we have to send it. 
           
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
			std::cout << "inputorweight:" << pck->get_data() << std::endl;
            pck_to_send.push_back(pck); //storing into the vector data type structure used in class Connection 
            this->read_connections[i]->send(pck_to_send); //Sending the input or weight through the connection
            input_fifos[i]->pop(); //pulling from fifo
            

        }
//       std::cout << std::endl;   
    }


}

//TODO Remove this connection
void SparseSDMemory::receive() { //TODO control if there is no space in queue
	std::cout << "receive!"<<local_cycle << std::endl;
    if(this->write_connection->existPendingData()) {
		std::cout << "receive write_connection:";
        std::vector<DataPackage*> data_received = write_connection->receive();
        for(int i=0; i<data_received.size(); i++) {
            write_fifo->push(data_received[i]);
			std::cout << data_received[i]->get_data() <<std::endl;
        }
    }
    for(int i=0; i<write_port_connections.size(); i++) { //For every write port
        if(write_port_connections[i]->existPendingData()) {//一个端口可以有多个数据包
			std::cout << "receive write_port_connections" << i <<":";
            std::vector<DataPackage*> data_received = write_port_connections[i]->receive(); 		   	for(int i=0; i<data_received.size(); i++) {
                 write_fifo->push(data_received[i]);
            }
        }    
    }
}


void SparseSDMemory::printStats(std::ofstream& out, unsigned int indent) {
    out << ind(indent) << "\"SDMemoryStats\" : {" << std::endl; //TODO put ID
    this->sdmemoryStats.print(out, indent+IND_SIZE);
    out << ind(indent) << "}"; //Take care. Do not print endl here. This is parent responsability
}

void SparseSDMemory::printEnergy(std::ofstream& out, unsigned int indent) {
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

