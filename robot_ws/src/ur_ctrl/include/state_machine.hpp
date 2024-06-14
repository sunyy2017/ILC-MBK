#ifndef STATE_MACHINE_HPP_
#define STATE_MACHINE_HPP_
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <csignal>

int running_sm=1;
void handle_sm(int sig){
  if (sig==SIGINT){
    running_sm = 0;
  }
}

//State need state name and a function that need global params and return a state response
//once execute, this class need to give a response
class State{
public:
  int running;
  std::string State_name;
  std::string (*State_fun)(void);
  std::string State_resp;
  State(std::string State_name_,std::string (*State_fun_)(void)):
    State_name(State_name_),State_fun(State_fun_),State_resp(""){}
  ~State(){}
  void execute(){
    std::cout<<"[STATE_MACHINE]Now at state \""<<State_name<<"\""<<std::endl;
    State_resp=State_fun();
    std::cout<<"[STATE_MACHINE]response \""<<State_resp<<"\""<<std::endl;
  }
};
//State transform need former state name, a transform vector pointing to next state
class State_trans{
public:
  std::string Cur_state;
  std::vector<std::string> State_resp;
  std::vector<std::string> Next_state;
  State_trans(std::string Cur_state_,std::string State_resp_,std::string Next_state_):
    Cur_state(Cur_state_){
    State_resp.push_back(State_resp_);
    Next_state.push_back(Next_state_);
  }
  ~State_trans(){}
  void add_trans(std::string State_resp_,std::string Next_state_){
    State_resp.push_back(State_resp_);
    Next_state.push_back(Next_state_);
  }
};
//State machine need a init function to init all global params
//once running, it will find "BEGIN" state to start, find a response "END" to end
//add trans add one transform once
class State_mach{
public:
  std::vector<State> State_vector;
  std::vector<State_trans> Trans_vector;
  bool (*Init_all)(void);
  State_mach(bool (*Init_all_)(void)):
    Init_all(Init_all_){}
  ~State_mach(){}
  void Add_state(std::string State_name_,std::string (*State_fun_)(void)){
    State_vector.push_back(State(State_name_,State_fun_));
  }
  void Add_trans(std::string Cur_state_,std::string State_resp_,std::string Next_state_){
    bool insert_=false;
    for(std::vector<State_trans>::iterator iter=Trans_vector.begin();iter!=Trans_vector.end();iter++){
      if(iter->Cur_state==Cur_state_){
        iter->add_trans(State_resp_, Next_state_);
        insert_=true;
      }
    }
    if(!insert_){
      Trans_vector.push_back(State_trans(Cur_state_,State_resp_,Next_state_));
    }
  }
  void run(){
    bool init_succ=Init_all();
    signal(SIGINT,handle_sm);
    if(!init_succ){
      std::cout<<"[STATE_MACHINE]init failed! exit"<<std::endl;
      return;
    }
    std::vector<State>::iterator entrance;
    bool entr_exist=false;
    for(std::vector<State>::iterator iter=State_vector.begin();iter!=State_vector.end();iter++){
      if(iter->State_name==std::string("BEGIN")){
        entrance=iter;
        entr_exist=true;
        break;
      }
    }
    if(!entr_exist){
      std::cout<<"[STATE_MACHINE]could not find \"BEGIN\" state! exit"<<std::endl;
      return;
    }
    std::vector<State>::iterator next=entrance;
    next->execute();
    std::string next_state;
    while(next->State_resp!=std::string("END")&&running_sm!=0){
      for(std::vector<State_trans>::iterator iter=Trans_vector.begin();iter!=Trans_vector.end();iter++){
        if(iter->Cur_state==next->State_name){
          std::vector<std::string>::iterator tmp=std::find(iter->State_resp.begin(),iter->State_resp.end(),next->State_resp);
          int index = std::distance(iter->State_resp.begin(), tmp);
          next_state=iter->Next_state[index];
          break;
        }
      }
      for(std::vector<State>::iterator iter=State_vector.begin();iter!=State_vector.end();iter++){
        if(iter->State_name==next_state){
          next=iter;
          break;
        }
      }
      next->execute();
    }
  }
};
#endif
