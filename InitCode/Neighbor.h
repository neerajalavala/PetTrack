#define MAX_NEIGHBOR 12
#define PAST_RECORD_LEN 5

class Neighbor {
    public:

    uint8_t num_neighbors;

    uint8_t neighbor_id[MAX_NEIGHBOR];
    uint8_t num_hop2_neighbrs[MAX_NEIGHBOR];
    uint8_t hop2_neighbors[MAX_NEIGHBOR][MAX_NEIGHBOR];

    bool neighbor_loc_avail[MAX_NEIGHBOR];
    int neighbor_loc[MAX_NEIGHBOR][2];
    int dist_to_neighbor[MAX_NEIGHBOR];
    float neighbor_FP_PW[MAX_NEIGHBOR];
    
    uint8_t last_ten_resp[PAST_RECORD_LEN]= {0,0,0,0,0};;//stores how many times it has been INIT or RESP in the last 10 communications
    // 1-resp, 0-init

    char buff[200];

    //Add new record to last_ten
    void push_new(uint8_t is_resp)
    {
       uint8_t temp[PAST_RECORD_LEN-1];
       memcpy(temp, last_ten_resp, PAST_RECORD_LEN-1);
       last_ten_resp[0] = is_resp;
       memcpy(&last_ten_resp[1], temp, PAST_RECORD_LEN-1);
    }
    
    //returns my priority number
    uint8_t return_priority()
    {
      uint8_t count=0;
      for (int i=0;i<PAST_RECORD_LEN;i++)
      {
        count = count+last_ten_resp[i];
      }  
      return count;
    }

      
    //Check whether neighbor exists
    bool check_neighbor_exist(uint8_t new_node)
    {
      for(int i=0;i<num_neighbors;i++)
       {
          if(new_node == neighbor_id[i])
            return true;
       }  
       return false; 
    }

        //Check whether neighbor exists
    int find_neighbor_id(uint8_t new_node)
    {
      for(int i=0;i<num_neighbors;i++)
       {
          if(new_node == neighbor_id[i])
            return i;
       }  
       return -1;
    }

    //Add new neighbor
    void add_neighbor(uint8_t new_node)
    {
      if(!check_neighbor_exist(new_node))
      {
        
         neighbor_id[num_neighbors] = new_node;
         num_neighbors++;
         
//          sprintf(buff, "neighbor added ID %u total neighbors %u",new_node, num_neighbors);
//          Serial.println(buff);
      }
    }

    void add_neighbor(uint8_t new_node, int dist)
    {
      if(!check_neighbor_exist(new_node))
      {
          neighbor_id[num_neighbors] = new_node;        
          dist_to_neighbor[num_neighbors] = dist;
          num_neighbors++;
      }
    }

    //Add new neighbor
    void add_neighbor_PW(uint8_t new_node,float FP_PW)
    {
      if(!check_neighbor_exist(new_node))
      {
        
         neighbor_id[num_neighbors] = new_node;
         neighbor_FP_PW[num_neighbors] = FP_PW;
         num_neighbors++;
         
//          sprintf(buff, "neighbor added ID %u total neighbors %u",new_node, num_neighbors);
//          Serial.println(buff);
      }else{
        int id = find_neighbor_id(new_node);
         neighbor_FP_PW[id] = FP_PW;
        }
    }

    uint8_t next_INIT()
    {
      if(num_neighbors==0)
      return 0;
      
      int rand_idx = random(num_neighbors);
      return neighbor_id[rand_idx];  
    }

//    void print_info()
//    {
//      sprintf(buff, "Num neighbors: %u ID: %u %u %u",num_neighbors, neighbor_id[0],neighbor_id[1],neighbor_id[2]);
//        Serial.println(buff);
//      }
    
};
