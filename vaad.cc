/* This file was originally provided by Zhengming Li, as the author of VAAD paper. 
 It has been adapted by PADIV authors to meet the new goals pursued by their contribution
 All adapted parts have been marked with UC3M, and they are the only elements whose intellectual property belong to PADIV authors */

/* This file contains the implementation details of the LEAPER scheme */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>

#include "cmu-trace.h"
#include "random.h"
#include "mobilenode.h"
#include "locdbase.h"
#include "vaad.h"
   
int hdr_vaad::offset_;
  
#define BEACON_RESCHED                                                     \
       beacon_timer_->resched(bint_ + Random::uniform(2*bdesync_* bint_) - \
			      bdesync_ * bint_)

/* definition of Location Database */
inline double distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  double dis = sqrt((x1 - x2)*(x1 - x2) +
	      (y1 - y2)*(y1 - y2) +
	      (z1 - z2)*(z1 - z2));
  return dis; 
}

// VaadApp OTcl linkage class
class VaadHeaderClass : public PacketHeaderClass {
public: 
  VaadHeaderClass() : PacketHeaderClass("PacketHeader/Vaad", sizeof(hdr_vaad))
  {
    bind_offset(&hdr_vaad::offset_);
  }
} class_vaadhdr;

static class VaadClass:public TclClass
{
public:
  VaadClass():TclClass ("Agent/Vaad")
  {
  }
  TclObject *create (int, const char *const *)
  {
    return (new VaadAgent ());
  }
} class_vaad;

//constructor function
VaadAgent::VaadAgent(void) : Agent(PT_VAAD), beacon_timer_(NULL), tracetarget(NULL), status_(0), 
data_exp_int_(DATA_EXP_INTERVAL),data_con_int_(DATA_CON_INTERVAL),token_con_int_(TOKEN_CON_INTERVAL),
L(0), prob(0), ED(0), Di(0), prob_thrs(NULL),flag_(0),type_(0),base_id_(0),pkt_sent_(0),pkt_recv_(0),last_recv_id(-1), last_send_id(-1),
bint_(2.0), bdesync_(0.5),verbose_(1),drop_debug_(0),
ldb_(0), mn_(0), ifq_(0), port_dmux_(0),ID_(0),name(NULL),table(0), nextSeg(0), record(NULL), recordSize(100), next_hop_(-1)  
{ 
 // p_data = new data_buffer(MAX_DATA_ENTRY);
  
  bind_time("data_exp_int_", &data_exp_int_);
  bind_time("data_con_int_", &data_con_int_);
  bind_time("token_con_int_", &token_con_int_);
  bind("verbose_", &verbose_);
  bind("drop_debug_", &drop_debug_);  
  bind("off_vaad_", &off_vaad_);
  bind("base_pkt_id_", &base_id_);
  bind("L", &L);
  bind("longCad", &longCad);
  bind_time("prob", &prob);
  bind("ED", &ED);
  bind("ID_", &ID_);      
  bind("type_",&type_);
  bind("flag_", &flag_);
  /* In the command function of this class, these variables should be initialized accordingly. */
  name = new char[20];
  memset(name, 0, 20);
}

VaadAgent::~VaadAgent()
{
  if(!name) 
    delete [] name;
}
// OTcl command interpreter
int VaadAgent::command(int argc, const char*const* argv)
{

  if (argc == 2) {
    if (strcmp(argv[1], "start-vaad") == 0) {
      init();
      return TCL_OK;
    }
  }
  else if (argc == 3) {
    TclObject *obj;

    if (strcasecmp(argv[1], "tracetarget") == 0) {
      if ((obj = TclObject::lookup(argv[2])) == 0) {
	fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		argv[2]);
	return TCL_ERROR;
      }
      tracetarget = (Trace *) obj;
      return TCL_OK;
    }
    if (strcmp(argv[1], "install-tap") == 0) {
      if ((obj = TclObject::lookup(argv[2])) == 0) {
	fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		argv[2]);
	return TCL_ERROR;
      }
      Mac *m = (Mac *) obj;
      m->installTap(this);
      return TCL_OK;
    }
    if (strcmp(argv[1], "node") == 0) {
      if ((obj = TclObject::lookup(argv[2])) == 0) {
	fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		argv[2]);
	return TCL_ERROR;
      }
      //trace("Node is %s\n", argv[2]);
      mn_ = (MobileNode *) obj;
      //printf("have found node %s\n", argv[2]);
      return TCL_OK;
    }
    if (strcmp(argv[1], "ldb") == 0) {
      if ((obj = TclObject::lookup(argv[2])) == 0) {
	fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		argv[2]);
	return TCL_ERROR;
      }
      ldb_ = (LocDbase *) obj;
      return TCL_OK;
    }
    if (strcmp(argv[1], "add-ifq") == 0) {
      if ((obj = TclObject::lookup(argv[2])) == 0) {
	fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		argv[2]);
	return TCL_ERROR;
      }
      ifq_ = (PriQueue *) obj;
      return TCL_OK;
    }
	//set the type
	if(strcmp(argv[1],"set-type")==0)
	{
	  type_ = atoi(argv[2]);
          return TCL_OK;
	}
	//set the flag
	if(strcmp(argv[1],"set-flag")==0)
	{
	  flag_ = atoi(argv[2]);
          return TCL_OK;
	}
     if(strcmp(argv[1], "port-dmux") == 0)
    {
      if ((obj = TclObject::lookup(argv[2])) == 0) {
	fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		argv[2]);
	return TCL_ERROR;
      }
      port_dmux_ = (NsObject *) obj;
      return TCL_OK;
    }

      //record the name of itsefl
      if(strcmp(argv[1], "set-self")==0)
     {
        if((obj = TclObject::lookup(argv[2])) == 0) {
         fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		argv[2]);
         trace("Cannot find the agent %s\n", argv[2]);
         return TCL_ERROR;
        }
     //printf("my name is %s\n", argv[2]);
     strcpy(name, argv[2]);
     //printf("My own name is %s\n", name);
     
     return TCL_OK;
     }      
  }
  return (Agent::command (argc, argv));
}
/* Initialize the parameters in VaadApp */
void VaadAgent::init()
{
  beacon_timer_ = new Vaad_BeaconTimer(this);
  beacon_timer_->resched(BEACON_OFF_+bint_ + Random::uniform(2*bdesync_* bint_) - bdesync_ * bint_);
  
  ad_timer_ = new Vaad_AdTimer(this);

  //create the neighbor table
  table = new NeiTable(4.5*bint_);   
  //initialize the record for the simulation results.
  record = new recordStruct *[recordSize];
  int i;
  for(i=0; i<recordSize; i++)
    record[i]=new recordStruct;
  //for the virtual link
  memset(&my_ad, 0, sizeof(Ad_Info));

 // prob = 1-1.0*L/ED;  
  trace("I %d with prob %f L %d\n", ID_, prob, L);
}

/* Real procedures begin now! */
/* This function process the overheard packets, which may not be targeted at this node. */
/* At present, one node can only process one data packet at a certain time. Only after the  */
void VaadAgent::tap(const Packet *p)
{
  hdr_cmn *hdrc = HDR_CMN(p);
  hdr_ip *iph = HDR_IP(p);
  hdr_vaad *vaad_h = (hdr_vaad *) p->access(off_vaad_);
 // double stime = Scheduler::instance().clock();
  //trace("%f: node (%d) packet received!\n", stime, ID_);
  
  if(vaad_h->mode != DATA_PACKET && vaad_h->mode != BEACON && vaad_h->mode != EXP_RSP)
  {   
	trace("Get an irrevelant packet: mode %d, pkt_id %d, data %d!\n", vaad_h->mode, vaad_h->pkt_id, vaad_h->data);	
	return;
  }
  switch (vaad_h->mode)
  {	
	case DATA_PACKET:
	{
	  double stime = Scheduler::instance().clock();
	  double poss[3];
	  mn_->getLoc(&poss[0],&poss[1], &poss[2]);
	  ////uc3m - ready to comment it out!
	//uc3m  trace("at time %f: I %d at poss %f tap an Ad packet from %d !\n", stime, ID_, poss[0], hdrc->prev_hop_);
	  process_data_in(p,0);
	  break;
	}
        case BEACON:      
	{	 
             proc_beaconIn(p,0);             
             break;              
	}
	case EXP_RSP:
	{
	 
	  if(hdrc->next_hop_ == ID_ && vaad_h->src_id == next_hop_)
	  {
	    double stime = Scheduler::instance().clock();
	    double poss[3];
	    mn_->getLoc(&poss[0],&poss[1], &poss[2]);
	    if(ad_timer_->status()==TIMER_PENDING)
	    {
	     ad_timer_->cancel();
	    }
	    next_hop_ = -1;
	    retrans = -1;
	    trace("Time %f: I %d at pos %f receives the EXP RSP from %d, and cancel the ARQ timer!\n", stime, ID_, poss[0], hdrc->prev_hop_);
	     //UC3M - IF THE MESSAGE TO PROCESS NETWORK PARTITION
            if(vaad_h->net_partition = 1){
		vaad_h->net_partition = 0;
                trace("Network partition detected! \n");
		send_new_neighbour(p);
		break;
	  }
	  }
	  break;
	}
	default:
		//Trace error;
		trace("Unknown \n");
		break;
  }    
}
/* This function process the neighbor table */
int  VaadAgent::proc_beaconIn(const Packet * p, int flag = 0)
{
  hdr_vaad * header = (hdr_vaad *) p->access(off_vaad_);
  NeiEntity ent;

  ent.dst = header->src_id;
  ent.x = header->source_pos[0];
  ent.y = header->source_pos[1];
  ent.z = header->source_pos[2];

  ent.latest_time = Scheduler::instance().clock();
  strcpy(ent.name, header->src_name);
  table->add_ent(&ent);
  //trace("I (%d) have %d neighbors!\n", ID_, table->count());
  return 0;
}

void VaadAgent::process_data_in(const Packet *p, int flag)
{
	hdr_cmn *hdrc = HDR_CMN(p);
	hdr_ip *iph = HDR_IP(p);//(hdr_ip *)p->access(off_ip_);
        hdr_vaad *vaad_h = (hdr_vaad *) p->access(off_vaad_);
	
	//Check the general information of this data packet.
	double myx,myy,myz;
	double dist_rel=0, dist_src = 0;
        mn_->getLoc(&myx,&myy,&myz);
	dist_rel = distance(vaad_h->rel_pos[0],vaad_h->rel_pos[1],vaad_h->rel_pos[2], 
		vaad_h->source_pos[0], vaad_h->source_pos[1], vaad_h->source_pos[2]);
        dist_src = distance(myx,myy,myz, vaad_h->source_pos[0], vaad_h->source_pos[1], vaad_h->source_pos[2]);
	
	int forMe=0;
	int impRSP = 0;
	if(hdrc->next_hop_==ID_)
	  forMe=1;
	if(hdrc->prev_hop_ == next_hop_)    //Implicit RSP from the next hop.
	  impRSP = 1;
	//trace("I %d, next_hop_ %d, src_id %d\n", ID_, next_hop_, hdrc->prev_hop_);
	    
	if(forMe==0 && !impRSP)  //happen to overhear an Ad
	{
	  if(last_recv_id < vaad_h->pkt_id)    //fresh ad.
	  {
	    last_recv_id = vaad_h->pkt_id;
	    //MobileNode *smn=ldb_->nlookup(vaad_h->src_id);
	    //smn->updateNumReceiver(last_recv_id);
	    VaadAgent * vagent=NULL;
	    TclObject * obj=NULL;
	    if((obj=TclObject::lookup(vaad_h->src_name))==0)
	    {
	      trace("I %d cannot find the neighbor !\n", ID_);
	      return;
	    }
	    vagent=(VaadAgent *)obj;
	    vagent->updateNumReceiver(last_recv_id);
	    //uc3m - ready to comment it out!
	  //uc3m  trace("I %d overhear a new ad packet !\n", ID_);
	  }
	  return;
	}
	else if (impRSP)
	{
	  //cancel the timer and reset the data packet
	  if(ad_timer_->status()==TIMER_PENDING)
	  {
	   ad_timer_->cancel();
	  }
	   next_hop_ = -1;
	   retrans = -1;
	   trace("I %d receives the implicit RSP, and cancel the ARQ timer!\n", ID_);	  
	}
	else if(forMe)
	{
	  trace("I %d receive a packet for me! \n", ID_);	
	  if(last_send_id < vaad_h->pkt_id)
	  {	   
	    last_send_id = vaad_h->pkt_id;
	    last_recv_id = vaad_h->pkt_id;
	    proc_data_out(p);	    
	  }
	  else   //ASK for ARQ
	  {
		//UC3M - I RECEIVE A PACKET THAT I HAD ALREADY PREVIOUSLY RECEIVED. THE PREVIOUS NODE IS ASKING FOR EACK
	    last_recv_id = vaad_h->pkt_id;
	    send_exp_arq(vaad_h);    //send exp_arq to the node
	  }
	  last_recv_id = vaad_h->pkt_id;
	}	
}

void VaadAgent::proc_data_out(const Packet *p)
{
  hdr_cmn *hdrc = HDR_CMN(p);
  hdr_ip *iph = HDR_IP(p);//(hdr_ip *)p->access(off_ip_);
  hdr_vaad *vaad_h = (hdr_vaad *) p->access(off_vaad_);
  
  //first of all, try to find the next hop 
  double my_pos[3];
  mn_->getLoc(&my_pos[0], &my_pos[1], &my_pos[2]);
  //UC3M - CHANGED NeiEntity *nei= findNextHop(vaad_h->source_pos, my_pos, vaad_h->L);
   //this was from UC3M, to propose an enhanced findNeighbour NeiEntity *nei= findNextHopUC3M(vaad_h->source_pos, my_pos, vaad_h->hop_num);
NeiEntity *nei= findNextHopUC3M(vaad_h->source_pos, my_pos, 1);
 
  double poss[3];
  mn_->getLoc(&poss[0], &poss[1], &poss[2]);
  if(nei==NULL)
  {    
    trace("I %d at x %f, cannot send out the packet %d - no neighbour\n", ID_, poss[0], last_recv_id);
    //UC3M - TO DISTINGUISH WHEN PACKET IS NOT SENT IN ABSENCE OF NEIGHBOURS
    char out[27];      // Prepare the output to the Tcl interpreter.
    strcpy(out, "countFeedbackNoNeighbours");
    out[26]='\0';				
    Tcl& tcl = Tcl::instance();
    tcl.eval(out);
    //END UC3M
    VaadAgent * vagent=NULL;
    TclObject * obj=NULL; 
    vagent=(VaadAgent *)obj;    
    //vagent->recordStop(last_recv_id, 0, 0, nei==NULL);
    trace("Sending exp arq... \n");
	//UC3M - THIS IS THE NETWORK PARTITION CASE
	vaad_h->net_partition = 1;
    send_exp_arq(vaad_h);
    return;
  }
  //This is to ensure that the packet will not be bounced around at the end of x axis.
  if(nei!=NULL && poss[0]>=MAX_X_-50)
  {
    trace("I %d at x %f, cannot send out the packet %d due to reason 3!\n", ID_, poss[0], last_recv_id);
    VaadAgent * vagent=NULL;
    TclObject * obj=NULL;
    if((obj=TclObject::lookup(vaad_h->src_name))==0)
    {
	trace("I %d cannot find the neighbor !\n", ID_);
	return;
    }
    vagent=(VaadAgent *)obj;    
    vagent->recordStop(last_recv_id, 0, 0, nei==NULL);
     // UC3M - THIS IS NOT WRITTEN IN THE PAPER
    send_exp_arq(vaad_h);
    return;    
  }
  
  //send this to the next hop.
  // UC3M - check if it is possible to re-send the packet
	double myx,myy,myz;
	double dist_src = 0;
        mn_->getLoc(&myx,&myy,&myz);
	dist_src = distance(myx,myy,myz, vaad_h->source_pos[0], vaad_h->source_pos[1], vaad_h->source_pos[2]);
	int newLength = 0;
	char out[32];      // Prepare the output to the Tcl interpreter.
	trace("distance %f \n", dist_src);
	//if(dist_src >= DEFAULT_UC3M_ADCHAIN_DISTANCE){
	if(dist_src >= L){
		trace("I %d at x %f, cannot send out the packet %d . Maximum distance reached!! \n", ID_, poss[0], last_recv_id);
		trace("TODO - SEND TO RSU ... \n");
		strcpy(out, "countFeedbackDistance");
		out[22]='\0';				
	        Tcl& tcl = Tcl::instance();
	        tcl.eval(out);
		//do not re-send the packet!
		return;
	}else{
		double random = Random::uniform(1.0);
		
		//UC3M- CHANGED if(random<=DEFAULT_UC3M_PROB_TAKE_AD){
			if(random<=prob){
			// OK - I wanna take the packet
			if(vaad_h->longCad == 1){
				trace("I %d at x %f, cannot send out the packet %d . I got the last coupon! \n", ID_, poss[0], last_recv_id);
				//trace("TODO - SEND TO RSU ... \n");
				strcpy(out, "countFinishedChain");
				out[26]='\0';				
			        Tcl& tcl = Tcl::instance();
			        tcl.eval(out);
				//do not re-send the packet!
				return;
			}else{
				// I take  the coupon and send the remaining ones
				newLength = vaad_h->longCad - 1;
				trace("I %d at x %f, ready to send out the packet %d after taking coupon. %d remaining \n", ID_, poss[0], last_recv_id, newLength);
				 // call the "recv" function defined in your TCL script
				strcpy(out, "countCouponsTaken");
				out[26]='\0';				
			        Tcl& tcl = Tcl::instance();
			        tcl.eval(out);
				
		
			}
		}else{
			//UC3M - no, I don't wanna take the packet...
			newLength = vaad_h->longCad;
			trace("I %d at x %f, ready to send out the packet %d without taking coupon. \n", ID_, poss[0], last_recv_id);
			
			strcpy(out, "countCouponsNotTaken");
			out[26]='\0';	

		        Tcl& tcl = Tcl::instance();
		        tcl.eval(out);

		}
	}

  	//send this to the next hop.
  Packet *p2 = allocpkt();
  hdr_ip *iph2 = HDR_IP(p2);
  hdr_cmn *hdrc2 = HDR_CMN(p2);
  hdr_vaad *vaadh2 = (hdr_vaad *) p2->access(off_vaad_);

  // set up header
  next_hop_ = nei->dst;    //record the next hop in this node
  hdrc2->prev_hop_=ID_;
  hdrc2->next_hop_ = nei->dst;
  hdrc2->addr_type_ = 0;//AF_INET;
  hdrc2->ptype_ = PT_VAAD;
  iph2->daddr() = Address::instance().create_ipaddr(nei->dst, VAAD_PORT);
  iph2->dport() = VAAD_PORT;//0;
  iph2->ttl() = iph->ttl()-1;
  iph2->src() = iph->src();

  memcpy(vaadh2, vaad_h, sizeof(hdr_vaad));
  vaadh2->hop_num = vaad_h->hop_num+1;  
  vaadh2->src_id = ID_;
  // UC3M - SET THE ADCHAIN LENGTH
  vaadh2->longCad = newLength;
  mn_->getLoc(&vaadh2->rel_pos[0], &vaadh2->rel_pos[1], &vaadh2->rel_pos[2]);
  //vaadh2->rel_pos[0]=nei->x;
  //vaadh2->rel_pos[1]=nei->y;
  //vaadh2->rel_pos[2]=nei->z;  
  hdrc2->size() = vaadh2->size() + IP_HDR_LEN;
  target_->recv(p2, (Handler *)0);  //send it out right now.
  trace("I %d at x %f, relay a data packet to %d\n", ID_,vaadh2->rel_pos[0], nei->dst);
  //update the receiver and relayer of this node
  //MobileNode *smn=ldb_->nlookup(vaad_h->src_id);
  VaadAgent * vagent=NULL;
  TclObject * obj=NULL;
  if((obj=TclObject::lookup(vaad_h->src_name))==0)
  {
      trace("I %d cannot find the neighbor !\n", ID_);
      return;
  }
    vagent=(VaadAgent *)obj;    
  vagent->updateNumReceiver(vaad_h->pkt_id);  
  vagent->updateNumRelayer(vaad_h->pkt_id);
  
  //make a virtual link to the destination node
/*  VaadAgent * vagent2=NULL;
  TclObject * obj2=NULL;
  if((obj2=TclObject::lookup(nei->name))==0)
  {
      trace("I %d cannot find the neighbor !\n", ID_);
      return;
   }
    vagent2=(VaadAgent *)obj2;    
  vagent2->ad_pending(p2);  */

  //Now begin to set the timer for the next hop.
  retrans = MAX_ARQ_-1;
  ad_timer_->resched(0.5*ARQ_T_+Random::uniform(ARQ_T_));    //begin the ARQ timer
  ad_pending(p2);  //save a copy of the ad packet.
  return;
}

/* UC3M - SEND A PACKET TO OTHER NODE AFTER FAILED RETRANSMISSIONS. ADAPTED FROM THE ORIGINAL proc_data_out */
void VaadAgent::send_new_neighbour(const Packet *p)
{
  hdr_cmn *hdrc = HDR_CMN(p);
  hdr_ip *iph = HDR_IP(p);//(hdr_ip *)p->access(off_ip_);
  hdr_vaad *vaad_h = (hdr_vaad *) p->access(off_vaad_);
  
  //first of all, try to find the next hop 
  double my_pos[3];
  mn_->getLoc(&my_pos[0], &my_pos[1], &my_pos[2]);
  //UC3M - CHANGEDNeiEntity *nei= findNextHop(vaad_h->source_pos, my_pos, vaad_h->L);
   //this was from UC3M, to propose an enhanced way of finding neighbour. It was not justified: NeiEntity *nei= findNextHopUC3M(vaad_h->source_pos, my_pos, vaad_h->hop_num);
NeiEntity *nei= findNextHopUC3M(vaad_h->source_pos, my_pos, 1);
 
  double poss[3];
  mn_->getLoc(&poss[0], &poss[1], &poss[2]);
  if(nei==NULL)
  {    
    trace("I %d at x %f, cannot send out the packet %d - no neighbour\n", ID_, poss[0], last_recv_id);
    //UC3M - TO DISTINGUISH WHEN PACKET IS NOT SENT IN ABSENCE OF NEIGHBOURS
    char out[27];      // Prepare the output to the Tcl interpreter.
    strcpy(out, "countFeedbackNoNeighbours");
    out[26]='\0';				
    Tcl& tcl = Tcl::instance();
    tcl.eval(out);
    //END UC3M
    VaadAgent * vagent=NULL;
    TclObject * obj=NULL; 
    vagent=(VaadAgent *)obj;    
    //vagent->recordStop(last_recv_id, 0, 0, nei==NULL);
    trace("Sending exp arq... \n");
	//UC3M - THIS IS THE NETWORK PARTITION CASE
    vaad_h->net_partition = 1 ;
    send_exp_arq(vaad_h);
    return;
  }
  //This is to ensure that the packet will not be bounced around at the end of x axis.
  if(nei!=NULL && poss[0]>=MAX_X_-50)
  {
    trace("I %d at x %f, cannot send out the packet %d due to reason 3!\n", ID_, poss[0], last_recv_id);
    VaadAgent * vagent=NULL;
    TclObject * obj=NULL;
    if((obj=TclObject::lookup(vaad_h->src_name))==0)
    {
	trace("I %d cannot find the neighbor !\n", ID_);
	return;
    }
    vagent=(VaadAgent *)obj;    
    vagent->recordStop(last_recv_id, 0, 0, nei==NULL);
     // UC3M - THIS IS NOT WRITTEN IN THE PAPER
    send_exp_arq(vaad_h);
    return;    
  }
  
  //send this to the next hop.
  // UC3M - check if it is possible to re-send the packet
	double myx,myy,myz;
	double dist_src = 0;
        mn_->getLoc(&myx,&myy,&myz);
	dist_src = distance(myx,myy,myz, vaad_h->source_pos[0], vaad_h->source_pos[1], vaad_h->source_pos[2]);
	int newLength = 0;
	char out[32];      // Prepare the output to the Tcl interpreter.
	trace("distance %f \n", dist_src);
	//if(dist_src >= DEFAULT_UC3M_ADCHAIN_DISTANCE){
	if(dist_src >= L){
		trace("I %d at x %f, cannot send out the packet %d . Maximum distance reached!! \n", ID_, poss[0], last_recv_id);
		trace("TODO - SEND TO RSU ... \n");
		strcpy(out, "countFeedbackDistance");
		out[22]='\0';				
	        Tcl& tcl = Tcl::instance();
	        tcl.eval(out);
		//do not re-send the packet!
		return;
	}
			
		

  	//send this to the next hop.
  Packet *p2 = allocpkt();
  hdr_ip *iph2 = HDR_IP(p2);
  hdr_cmn *hdrc2 = HDR_CMN(p2);
  hdr_vaad *vaadh2 = (hdr_vaad *) p2->access(off_vaad_);

  // set up header
  next_hop_ = nei->dst;    //record the next hop in this node
  hdrc2->prev_hop_=ID_;
  hdrc2->next_hop_ = nei->dst;
  hdrc2->addr_type_ = 0;//AF_INET;
  hdrc2->ptype_ = PT_VAAD;
  iph2->daddr() = Address::instance().create_ipaddr(nei->dst, VAAD_PORT);
  iph2->dport() = VAAD_PORT;//0;
  iph2->ttl() = iph->ttl()-1;
  iph2->src() = iph->src();

  memcpy(vaadh2, vaad_h, sizeof(hdr_vaad));
  vaadh2->hop_num = vaad_h->hop_num+1;  
  vaadh2->src_id = ID_;
  // UC3M - SET THE ADCHAIN LENGTH
  vaadh2->longCad = vaad_h->longCad;
  vaadh2->net_partition = vaad_h->net_partition;		
  mn_->getLoc(&vaadh2->rel_pos[0], &vaadh2->rel_pos[1], &vaadh2->rel_pos[2]);
  //vaadh2->rel_pos[0]=nei->x;
  //vaadh2->rel_pos[1]=nei->y;
  //vaadh2->rel_pos[2]=nei->z;  
  hdrc2->size() = vaadh2->size() + IP_HDR_LEN;
  target_->recv(p2, (Handler *)0);  //send it out right now.
  //update the receiver and relayer of this node
  //MobileNode *smn=ldb_->nlookup(vaad_h->src_id);
  VaadAgent * vagent=NULL;
  TclObject * obj=NULL;
  if((obj=TclObject::lookup(vaad_h->src_name))==0)
  {
      trace("I %d cannot find the neighbor !\n", ID_);
      return;
  }
    vagent=(VaadAgent *)obj;    
  vagent->updateNumReceiver(vaad_h->pkt_id);  
  vagent->updateNumRelayer(vaad_h->pkt_id);
  
  //make a virtual link to the destination node
/*  VaadAgent * vagent2=NULL;
  TclObject * obj2=NULL;
  if((obj2=TclObject::lookup(nei->name))==0)
  {
      trace("I %d cannot find the neighbor !\n", ID_);
      return;
   }
    vagent2=(VaadAgent *)obj2;    
  vagent2->ad_pending(p2);  */

  //Now begin to set the timer for the next hop.
  retrans = MAX_ARQ_-1;
  ad_timer_->resched(0.5*ARQ_T_+Random::uniform(ARQ_T_));    //begin the ARQ timer
  ad_pending(p2);  //save a copy of the ad packet.
  return;
}



/* This function process the data packet to be sent out  */
void VaadAgent::recv(Packet *p, Handler *h)
{  
  //process the data packets from the upper layer
  hdr_cmn *hdrc = HDR_CMN(p);
  hdr_vaad *vaad_h = (hdr_vaad *) p->access(off_vaad_);
  hdr_ip *iph = HDR_IP(p);//(hdr_ip *) p->access(off_ip_);
  //Currently I don't know what's the correct way of getting the address.
  ///TBD in the future
  int src = Address::instance().get_nodeaddr(iph->src_.addr_);
  int dst = Address::instance().get_nodeaddr(iph->dst_.addr_);  
    
  if ((src == mn_->address()) &&
      (hdrc->num_forwards() == 0))
  //This is the data packet from the upper layer, and should be sent out by this node.
  //Assuming that the destination ID has been filled.
  {
    //Construct the INIT_DATA packet.
        double stime = Scheduler::instance().clock();
	//trace out the data packet
	trace("%f: I(%d) Receive a data packet from the upper layer: src %d, dst %d!\n", stime, ID_, src, dst);
	//trace("I %d have neighbors %d\n", ID_, table->count());
	//We need to check all the neighbors, in order to figure out the proper installment
	// first determine the Di
	/* COMMENTED BY UC3M        
	if(flag_ == 1 && !prob_thrs)    //the source RSU. 
	  {
	     prob_thrs = new double [MAX_PROB];
	     int i=0;
	     if(prob > 0)
	     {
	     for(i=0;i<MAX_PROB; i++)
	     {
		prob_thrs[i] = (1-prob)*pow(prob, i);
	     }
	     }
	     else
	     {
		prob_thrs[0]=1;
	     for(i=1;i<MAX_PROB; i++)
	     {
		prob_thrs[i] = 0;
	     }
	     }
	  }
	double cur_prob = Random::uniform(1.0);
        int k;
	for(k=0;k<MAX_PROB; k++)
	{
	   cur_prob -= prob_thrs[k];
 	   if(cur_prob <=0)
		break;
	}
	Di = (k+1)*L;
      //uc3m  trace("I %d as the source determines Di as %d, cur_prob %f\n", ID_, Di, cur_prob);
	*/
	
        double src_pos[3];
	mn_->getLoc(&src_pos[0], &src_pos[1], &src_pos[2]);
	//Di = L added by UC3M for retrocompatibility when using the old findNextHop
	
	//UC3M -        NeiEntity *nei= findNextHop(src_pos, src_pos, Di);
         NeiEntity *nei= findNextHopUC3M(src_pos, src_pos, 1);
	/*old code
	int toSend=0;
	if(nextSeg==0 && nei!=NULL)
	  toSend=1;
	if(nextSeg == 1)
	{	  
	    toSend=0;
	}
	if(toSend==0)
	{
	*/ 
	if(nei==NULL)
	{
	  trace("I %d cannot find a neighbor for myself!\n", ID_);
	  recordStop(base_id_+pkt_sent_, 0, 0, nei==NULL);
	  return;
	}	
	
        //trace("Pkt type %d, addr_type %d, direction_ %d\n", hdrc->ptype_, hdrc->addr_type_, hdrc->direction_);
        //trace("Dst IP %d, Dst port %d\n", iph->dst_.addr_, iph->dst_.port_);
	hdrc->size() += IP_HDR_LEN + 8 ;    
        hdrc->size()+=vaad_h->size();
        // iph->ttl_ = IP_DEF_TTL;
        iph->ttl_ = 128;
        //set up the common header
        hdrc->addr_type() = 0;
        //hdrc->next_hop_ = IP_BROADCAST;
	hdrc->prev_hop_ = ID_;
	hdrc->next_hop_ = nei->dst;
        hdrc->ptype_ = PT_VAAD;
        //set up the IP address
        //iph->daddr() = Address::instance().create_ipaddr(IP_BROADCAST, LEAPER_PORT);;
	iph->daddr() = Address::instance().create_ipaddr(nei->dst, VAAD_PORT);
        iph->dport() = VAAD_PORT;
	
	mn_->getLoc(&vaad_h->source_pos[0], &vaad_h->source_pos[1], &vaad_h->source_pos[2]);
	mn_->getLoc(&vaad_h->rel_pos[0], &vaad_h->rel_pos[1], &vaad_h->rel_pos[2]);
	vaad_h->mode = DATA_PACKET;
	vaad_h->L = Di;
        vaad_h->prob = prob;	
        vaad_h->pkt_id = base_id_+pkt_sent_;
	vaad_h->src_id = ID_;
	pkt_sent_++;
        //last_recv_id ++;
        trace("I %d: initiate a data packet pkt_id %d, base_id_ %d, pkt_sent_ %d\n", ID_, vaad_h->pkt_id, base_id_, pkt_sent_);      
	trace("I %d: next hop is %d\n", ID_, nei->dst);
	//UC3M - COUNT COUPONS SENT
	char out[27];
	strcpy(out, "countCouponsSent");
	out[26]='\0';				
	Tcl& tcl = Tcl::instance();
        tcl.eval(out);
	//UC3M - END COUNT COUPONS SENT
        vaad_h->hop_num = 1;
        vaad_h->data = DEFAULT_DATA_CONTENT;
	//vaad_h->longCad = DEFAULT_UC3M_ADCHAIN_LENGTH;
        vaad_h->longCad = longCad;
        vaad_h->net_partition = 0;
	strcpy(vaad_h->src_name, name);
        //add the data myself
      //  proc_InitData(p, 1);
	target_->recv(p, (Handler *)0);
	return;
  }  
  //Tap has already handled the data packet. So we can drop it now.
  else
  {
   //drop(p, DROP_RTR_ROUTE_LOOP);
   Packet::free(p);
  // trace("I(%d) receives a data packet which has been Tapped!\n", ID_);
  }
  //For the data packets or other packets, call the Tap() function to process them.  
}

/* Callback functions */
/* send out a beacon packet */
void VaadAgent::beacon_callback()
{
  //Scheduler &s = Scheduler::instance();

  Packet *p = makeAlive();

  // schedule the transmission of this beacon
  if (p) {
    assert (!HDR_CMN(p)->xmit_failure_);
    //s.schedule (target_, p, 0);
    target_->recv(p,(Handler *)0);
  }
  // schedule the next beacon generation event
  BEACON_RESCHED;
}

//This is the callback function for the retransmission function
void VaadAgent::ad_callback()
{
  //first of all, trace;
  double stime = Scheduler::instance().clock();
  double my_pos[3];
  mn_->getLoc(&my_pos[0],&my_pos[1], &my_pos[2]);
  
 
  //first of all, try to find the next hop 
  hdr_vaad * vaad_h = &my_ad.vhdr;  
 

//send this to the next hop.
  Packet *p2 = allocpkt();
  hdr_ip *iph2 = HDR_IP(p2);
  hdr_cmn *hdrc2 = HDR_CMN(p2);
  hdr_vaad *vaadh2 = (hdr_vaad *) p2->access(off_vaad_);

  // set up header
  hdrc2->prev_hop_=ID_;
  hdrc2->next_hop_ = next_hop_;
  hdrc2->addr_type_ = 0;//AF_INET;
  hdrc2->ptype_ = PT_VAAD;
  iph2->daddr() = Address::instance().create_ipaddr(next_hop_, VAAD_PORT);
  iph2->dport() = VAAD_PORT;//0;
  iph2->ttl() = my_ad.ip_ttl;
  iph2->src() = my_ad.ip_sr;

  memcpy(vaadh2, vaad_h, sizeof(hdr_vaad));
  vaadh2->src_id = ID_;
  //vaadh2->hop_num = vaad_h->hop_num+1;  
  //mn_->getLoc(&vaadh2->rel_pos[0], &vaadh2->rel_pos[1], &vaadh2->rel_pos[2]);
  //vaadh2->rel_pos[0]=nei->x;
  //vaadh2->rel_pos[1]=nei->y;
  //vaadh2->relint_pos[2]=nei->z;  
  hdrc2->size() = vaadh2->size() + IP_HDR_LEN;
  target_->recv(p2, (Handler *)0);  //send it out right now.
  trace("I %d at x %f, retransmit a data packet to %d\n", ID_,vaadh2->rel_pos[0], next_hop_);
  //update the receiver and relayer of this node
  //MobileNode *smn=ldb_->nlookup(vaad_h->src_id);
 
   
   //update the transmission number and start the timer again
   retrans --;
   //UC3M - CHANGED TO ADD WHAT TO DO WHEN RETRANS = 0 --> FIND NEW NEIGHBOUR!
   if(retrans >=0){   
     ad_timer_->resched(0.5*ARQ_T_+Random::uniform(ARQ_T_));
   }else{
	send_new_neighbour(p2);
   }
  return;  
}
//This is a callback function to add the virtual ad packet.
void VaadAgent::ad_pending(Packet *p)
{
  hdr_cmn *hdrc = HDR_CMN(p);
  hdr_ip *iph = HDR_IP(p);//(hdr_ip *)p->access(off_ip_);
  hdr_vaad *vaad_h = (hdr_vaad *) p->access(off_vaad_);
  //first of all, check whether timer is still running or not;
 // trace("I %d receivs virtually a packet from %d\n", ID_, hdrc->prev_hop_);
  if(ad_timer_->status()!=TIMER_PENDING)
  {
    trace("Error: I %d do not have ad timer still running!\n", ID_);
    return;
  }
  //record the necessary information;
  memcpy(&my_ad.vhdr, vaad_h, sizeof(hdr_vaad));
  my_ad.ip_sr = iph->src();
  my_ad.ip_ttl = iph->ttl();
  my_ad.pre_hop = hdrc->prev_hop_;
  //start the timer.  for 10ms
 // ad_timer_->resched(0.01);
}

void VaadAgent::send_exp_arq(hdr_vaad *vaad_h)
{
  if(!vaad_h)
  {
    trace("Error: I %d receive a void vaad header!\n", ID_);
    return;
  }
  
  //try to create a new packet.
  Packet *p2 = allocpkt();
  hdr_ip *iph2 = HDR_IP(p2);
  hdr_cmn *hdrc2 = HDR_CMN(p2);
  hdr_vaad *vaadh2 = (hdr_vaad *) p2->access(off_vaad_);

  // set up header
  hdrc2->prev_hop_=ID_;
  hdrc2->next_hop_ = vaad_h->src_id; 
  hdrc2->addr_type_ = 0;//AF_INET;
  hdrc2->ptype_ = PT_VAAD;
  iph2->daddr() = Address::instance().create_ipaddr(vaad_h->src_id, VAAD_PORT);
  iph2->dport() = VAAD_PORT;//0;
  iph2->ttl() = my_ad.ip_ttl;
  iph2->src() = my_ad.ip_sr;

  memcpy(vaadh2, vaad_h, sizeof(hdr_vaad));
  vaadh2->mode = EXP_RSP;
  vaadh2->src_id = ID_;
  //uC3M
  vaadh2->longCad = vaad_h->longCad;
  vaadh2->net_partition = vaad_h->net_partition; 
  vaadh2->hop_num = vaad_h->hop_num;
  hdrc2->size() = vaadh2->size() + IP_HDR_LEN;
  target_->recv(p2, (Handler *)0);  //send it out right now.
  double stime = Scheduler::instance().clock();
  double my_pos[3];
  mn_->getLoc(&my_pos[0],&my_pos[1], &my_pos[2]);
  trace("Time %f: I %d at x %f, send an exp_rsq to node %d\n", stime,ID_,my_pos[0], vaad_h->src_id);
}

/* Definition of Trace functions */
void VaadAgent::trace(char *fmt,...)
{
  va_list ap;
  if (!tracetarget)
    return;

  va_start(ap, fmt);
  vsprintf(tracetarget->pt_->buffer(), fmt, ap);
  //printf.
  printf(tracetarget->pt_->buffer());
  tracetarget->pt_->dump();
  va_end(ap);
}

void VaadAgent::tracepkt(Packet *p, double now, int me, const char *type)
{
  char buf[1024];

  //hdr_vaad *vaadh = hdr_vaad::access(p);

  snprintf (buf, 1024, "V%s %.5f _%d_:", type, now, me);
  
    if (verbose_)
      trace("%s", buf);  
}

// construct and return an alive beacon packet
Packet *VaadAgent::makeAlive(void)
{
  Packet *p = allocpkt();
  hdr_ip *iph = HDR_IP(p);
  hdr_cmn *hdrc = HDR_CMN(p);
  hdr_vaad *vaadh = (hdr_vaad *) p->access(off_vaad_);

  // set up header
  hdrc->next_hop_ = IP_BROADCAST;
  hdrc->addr_type_ = AF_INET;
  hdrc->ptype_ = PT_VAAD;
  iph->daddr() = Address::instance().create_ipaddr(IP_BROADCAST, VAAD_PORT);
  iph->dport() = VAAD_PORT;//0;
  vaadh->mode = BEACON;
  vaadh->hop_num = 1;
  vaadh->src_id = ID_;
  strcpy(vaadh->src_name, name);
  mn_->getLoc(&(vaadh->source_pos[0]), &(vaadh->source_pos[1]), &(vaadh->source_pos[2]));
  // simulated size: hdr_gpsr, IP header
  hdrc->size() = vaadh->size() + IP_HDR_LEN;
  return p;
}
//This function finds the next hop for any node at any position
//Here L is the distance Di
NeiEntity *VaadAgent::findNextHop(double *src_pos, double *my_pos, int L)
{
   //trace("entro en find");
   if(table->count()<=0)
   {  
	nextSeg=0;
	return NULL;
   }
   //trace("tengo vecinos");
   double delta_l=0, l=0;
   delta_l=distance(src_pos[0],src_pos[1],src_pos[2], my_pos[0], my_pos[1],my_pos[2]);
    //trace("tengo vecinos, L vale %d , delta_l vale %d", L, delta_l);
   l= L-delta_l;
    if(l<=10)    //less than 10 meters will be ignored
    {
	nextSeg=1;
	return NULL;
    }
   //trace("estoy a menos de 10 metros...");
   double max_distance =(l<=DEFAULT_RANGE-50) ? l: (DEFAULT_RANGE-50);   
   trace("I %d: L is %d max_distance is %f\n", ID_, L, max_distance);

   //find all neighbors from my position to L
   NeiEntity * nei=NULL;
   double dist1=0, dist2=0;
   NeiEntity *next1=NULL, *next2=NULL;
   table->resetIter();
   while((nei=table->Iterate())!=NULL)
   {     
     double nx, ny, nz;   
     MobileNode *nnei=ldb_->nlookup(nei->dst);
     nnei->getLoc(&nx, &ny, &nz);
     
     //vagent->updateNumReceiver(last_recv_id);     
     if(ny-my_pos[2]>=0.5 || ny-my_pos[2]<=-0.5)
       continue;
     if(nx-src_pos[0] < my_pos[0]-src_pos[0])   //only find the farther node.
       continue;
     double mid_dist=distance(nx, ny, nz, my_pos[0], my_pos[1], my_pos[2]);
     if(mid_dist <=max_distance && mid_dist>dist1)
     {         
       next1=nei;
       dist1 = mid_dist;
     }
     //if(mid_dist <=DEFAULT_RANGE && mid_dist>dist2)
      // next2=nei;
   }
   if(next1)
   {
     nextSeg=0;
  //uc3m   trace("I %d: max_distance is %f, dist1 is %f\n", ID_, max_distance, dist1);
     return next1;     
   }
   else
   {
     if(l <= DEFAULT_RANGE-50)    //to the end of L
	nextSeg = 1;
     return NULL;
    }
}

//UC3M - ADDED FOR NEW CALCULATION OF NEXT HOP. CRITERION: CHOOSE THE nth NEAREST NEIGHBOUR, WHERE n IS THE HOP COUNT OF THE PACKET AT STAKE
NeiEntity *VaadAgent::findNextHopUC3M(double *src_pos, double *my_pos, int hop_count)
{
   if(table->count()<=0)
   {  
	// uc3m - no neighbours
	trace("oh my god - no neighbours. . .\n");
	return NULL;
   }
  
   trace("Choosing the %d th closer neighbor among %d candidates\n", hop_count, table->count());
   NeiEntity * nei=NULL;
   
   NeiEntity * n_hop_closer=NULL;
   double max_distance =DEFAULT_RANGE-50;
   double currentCloserDist=max_distance; 
   int i =0;
   while (i<hop_count){
	   table->resetIter();
   	   double mid_dist=0;
  	 while((nei=table->Iterate())!=NULL)
  	 {     
  	  	 double nx, ny, nz;   
  	  	 MobileNode *nnei=ldb_->nlookup(nei->dst);
  	  	 nnei->getLoc(&nx, &ny, &nz);
  	  	 mid_dist=distance(nx, ny, nz, my_pos[0], my_pos[1], my_pos[2]);
	         //uc3m - for debugging trace("currentCloserDist %f mid_dist %f max_dist %f \n",currentCloserDist, mid_dist,max_distance);
	  	 if(nx-src_pos[0] > my_pos[0]-src_pos[0])   //only consider neighbours that are farther than me. 
	  	 {
			//trace ("It is farther \n");
			if(mid_dist < currentCloserDist && mid_dist <=max_distance && nei != n_hop_closer){
				currentCloserDist = mid_dist;
				n_hop_closer = nei;
				trace(" new currentCloserDist %f \n",currentCloserDist);

			}
	  	 }
	 }
	i++;
   }
   if(n_hop_closer !=NULL){
	trace(" Candidate hop will be... %d \n", n_hop_closer->dst);
   }
   return n_hop_closer;
}

//Stop Type: 1: due to network segment
//           0: due to random test
void VaadAgent::recordStop(int pkt_id, double dist, int hop, int stopType)
{
  trace("Recording stop... \n");
  if(pkt_id>=recordSize)
  {
    int i;
    recordStruct ** tmp = record;
    recordSize *=2;
    record = new recordStruct *[recordSize];
    bcopy(tmp, record, pkt_id*sizeof(recordStruct *));
    for (i=pkt_id; i<recordSize; i++)
      record[i]=new recordStruct();
    delete [] tmp; 
  }
  record[pkt_id]->dist=dist;
  record[pkt_id]->hop=hop;
  record[pkt_id]->type=stopType;
}

void VaadAgent::updateNumReceiver(int pkt_id)
{
  if(pkt_id<=0)
    return;
  if(pkt_id>=recordSize)
  {
    int i;
    recordStruct ** tmp = record;
    recordSize *=2;
    record = new recordStruct *[recordSize];
    bcopy(tmp, record, pkt_id*sizeof(recordStruct *));
    for (i=pkt_id; i<recordSize; i++)
      record[i]=new recordStruct();
    delete [] tmp;    
  }
  record[pkt_id]->numRec++;  
}

void VaadAgent::updateNumRelayer(int pkt_id)
{
  if(pkt_id<=0)
    return;
  if(pkt_id>=recordSize)
  {
    int i;
    recordStruct ** tmp = record;
    recordSize *=2;
    record = new recordStruct *[recordSize];
    bcopy(tmp, record, pkt_id*sizeof(recordStruct *));
    for (i=pkt_id; i<recordSize; i++)
      record[i]=new recordStruct();
    delete [] tmp;    
  }
  record[pkt_id]->numRel++;
}

	       
	       
static int NeighbEntCmp(const void *a, const void *b)
{
  nsaddr_t ia = ((const NeiEntity *) a)->dst;
  nsaddr_t ib = (*(const NeiEntity **) b)->dst;
  if (ia > ib) return 1;
  if (ib > ia) return -1;
  return 0;
}

/* implementation of the Neighbor Table class */
NeiTable::NeiTable(double period): Timeout(period)
{
    int i;
    num_ne = 0;
    max_ne = 50;
    Iter = 0;
	tab = new NeiEntity * [max_ne];
	for (i=0; i<max_ne; i++)
	{
		tab[i] = new NeiEntity;
	}
}
NeiTable::~NeiTable()
{
	int i;
	if(tab)
	{
	  for (i=0;i<max_ne;i++)
	  {
	    delete tab[i];
	  }
	  delete [] tab;
	}
}
int NeiTable::count()
{
  return num_ne;
}
NeiEntity *NeiTable::Iterate()
{
   if(Iter < num_ne)
	{
	    Iter ++;
		return tab[Iter-1];				
	}
	else
	{
        Iter = 0;
	return NULL;
	}
}
void NeiTable::resetIter()
{
  Iter = 0;
}

void NeiTable::add_ent(NeiEntity * ent)
{
  NeiEntity **pne, *owslot;
  int i, j, r, l;

  //first of all, check the obselete neighbors in this.
  double st = Scheduler::instance().clock();
  checkTiming(st, Timeout);

  if ((pne = (NeiEntity **) bsearch(ent, tab, num_ne,
				    sizeof(NeiEntity *), NeighbEntCmp))) 
  {
    // already in table; overwrite
    // make sure there is no pending timer
    i = pne - tab;
    //(*pne)->dnt.force_cancel();
    /* XXX overwriting table entry shouldn't affect when to probe this
                 perimeter */
    // (*pne)->ppt.force_cancel();
    // careful not to overwrite timers!
    (*pne)->dst = ent->dst;
    (*pne)->x = ent->x;
    (*pne)->y = ent->y;
    (*pne)->z = ent->z;
    (*pne)->latest_time = ent->latest_time;
	strcpy((*pne)->name, ent->name);
    return; //*pne;
  }
  // may have to grow table
  if (num_ne == max_ne) 
  {
    NeiEntity **tmp = tab;
    max_ne *= 2;
    tab = new NeiEntity *[max_ne];
    bcopy(tmp, tab, num_ne*sizeof(NeiEntity *));
    for (i = num_ne; i < max_ne; i++)
      tab[i] = new NeiEntity();
    delete[] tmp;
  }

  // binary search for insertion point
  if (num_ne == 0)
    i = 0;
  else {
    l = 0;
    r = num_ne - 1;
    while ((r - l) > 0) {
      if (ent->dst < tab[l + ((r-l) / 2)]->dst)
	r = l + ((r - l) / 2) - 1;
      else
	l += (r - l) / 2 + 1;
    }
    if (r < l)
      i = r+1;
    else
      // r == l
      if (ent->dst < tab[r]->dst)
	i = r;
      else
	i = r+1;
  }

  // slide subsequent entries forward
  if (i <= (num_ne - 1))
    owslot = tab[num_ne];
  j = num_ne-1;
  while (j >= i)
    {
     tab[j+1] = tab[j];
     j--;
    }
    
  // slam into table, without overwriting timers
  if (i <= (num_ne - 1))
    tab[i] = owslot;
  tab[i]->dst = ent->dst;
  tab[i]->x = ent->x;
  tab[i]->y = ent->y;
  tab[i]->z = ent->z;
  tab[i]->latest_time = ent->latest_time;
  strcpy(tab[i]->name, ent->name);
  // invalidate the perimeter that may be cached by this neighbor entry
  //tab[i]->perilen = 0;
  // XXX gross way to indicate entry is *new* entry, graph needs planarizing
  //tab[i]->live = -1;
  num_ne++;
  return ;//tab[i];
}

void NeiTable::del_ent(NeiEntity *ent)
{
  NeiEntity **pne, *owslot;
  int i, j;
  //first of all, check the obselete neighbors in this.
  double st = Scheduler::instance().clock();
  checkTiming(st, Timeout);

  if ((pne = (NeiEntity **) bsearch(ent, tab, num_ne,
				    sizeof(NeiEntity *), NeighbEntCmp))) 
  {
    i = pne - tab;
    // make sure no timers scheduled for this neighbor
    //(*pne)->dnt.force_cancel();
    //(*pne)->ppt.force_cancel();
    // slide any subsequent table entries backward
    if (i < (num_ne - 1))
      owslot = tab[i];
    for (j = i; j < num_ne - 1; j++)
      tab[j] = tab[j+1];
    if (i < (num_ne - 1))
      tab[num_ne-1] = owslot;
    num_ne--;
  }
}
/* If the latest_timing is smaller than curr - T, the neighbor is obsolete, and should be removed from the neighbor table. */
void NeiTable::checkTiming(double curr, double T)
{
  int i,j;
  int pos = 0;
  NeiEntity *mid = NULL;
  while(1)
  {  
   //printf("Running, pos = %d, num_ne = %d!\n", pos, num_ne);
  for(i=pos;i<num_ne; i++)  //check the next position.
  {
    if(tab[i]->latest_time + T <= curr)
    {
          //printf("Delete obselete neighbors!\n");
	  mid = tab[i];
	  for(j=i;j<num_ne-1;j++)
	  {
	    tab[j] = tab[j+1];
	  }
	  tab[j] = mid;
	  num_ne --;	  
	  pos = i;
	  break;
     }
    // printf("I = = %d\n", i);
     pos = i;
  }
  if(pos >=num_ne-1)  //All have been checked.
      break;
  }
}
