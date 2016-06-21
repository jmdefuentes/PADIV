/* 
 This file was originally provided by Zhengming Li, as the author of VAAD paper. 
 It has been adapted by PADIV authors to meet the new goals pursued by their contribution
 All adapted parts have been marked with UC3M, and they are the only elements whose intellectual property belong to PADIV authors 
 */

/*
LEAPER is a scheme for faithful and reliable data packet relaying. This file contains the definitions of all the relevant classes and functions.
*/
#include "agent.h"
#include "ip.h"
#include "delay.h"
#include "scheduler.h"
#include "queue.h"
#include "trace.h"
#include "arp.h"
#include "ll.h"
#include "mac.h"
#include "priqueue.h"
#include "node.h"
#include "timer-handler.h"

//#define BSL_IMP       //For BSL Implementation.
#define VAAD_PORT 0xff
#define SOURCE_APP 1
#define DEFAULT_PACKET_INTER  0.200 //200 ms
#define DEFAULT_DATA_CONTENT  0x49495050
#define WRONG_DATA_CONTENT    0xccccdddd
#define DATA_PKT_LEN              36 //bytes
#define MAX_K                 10        //maximum K.
#define MAX_DATA_ENTRY        20
#define DEFAULT_RANGE         300      //meters
//define DEFAULT_UC3M_PROB_TAKE_AD         0.35      
//define DEFAULT_UC3M_ADCHAIN_LENGTH         10      
//define DEFAULT_UC3M_ADCHAIN_DISTANCE         800 
#define DATA_EXP_INTERVAL     5
#define DATA_CON_INTERVAL     0.01
#define TOKEN_CON_INTERVAL    0.01

#define PACKET_SIZE           20
#define BEACON_OFF_           700
#define MAX_X_                10000 //meters
#define MAX_ARQ_              3    //maximal number of retransmission
#define ARQ_T_                0.015 // 10ms for retransmission
#define MAX_PROB              100   //


enum PacketType{
DATA_PACKET = 0,
BEACON,
EXP_RSP
};

//class declarations
class Vaad_BeaconTimer; 
class NeiEntity;
class NeiTable;
class Ad_Info;
class Vaad_AdTimer;
class hdr_vaad;


class hdr_vaad {
public:
        static int offset_;
	int pkt_id;     // packet sequence number
	int mode;  //type of the packet: DATA, INIT, Single Token or Total Token
	unsigned long data;  //data content in the packet.
	int           L;               //road segment length; Now it is Di, instead of L
	double        prob;                // filtering probability; I don't think this will be used. 
	double source_pos[3];  //x, y, z	
	double rel_pos[3];   //x,y,z positions of the relayer node        
	unsigned long src_id;   //ID or the source node	
	unsigned long hop_num;  //Current hop number of this data packet
	unsigned int longCad; //UC3M - Amount of ads in this chain
        char    src_name[20];
        //UC3M - ADDED TO INDICATE NETWORK PARTITION
	int net_partition ; 
        inline static int& offset() { return offset_; }
        static hdr_vaad* access(const Packet* p) 
        {
                return (hdr_vaad*) p->access(offset_);
        }
        int size(){return (7*sizeof(int)+7*sizeof(double)+3*sizeof(unsigned long)+20*sizeof(char));}
       // {return (3*sizeof(int)+10*sizeof(double)+7*sizeof(unsigned long)+20*sizeof(char));}
	 
};

class Ad_Info {
  public:
    hdr_vaad vhdr;
    int ip_ttl;
    ns_addr_t ip_sr;
    int pre_hop;
};

class recordStruct{
  public:
    double dist;
    int    hop;
    int    type;
    int    numRec;
    int    numRel;
    recordStruct():dist(0), hop(-1), type(-1), numRec(0), numRel(0){};
    ~recordStruct(){};
};

class VaadAgent: public Tap, public Agent{
public:		
    VaadAgent();
  ~VaadAgent();
	void tap(const Packet *p);
        void beacon_callback(void);
	void ad_callback(void);
	void ad_pending(Packet *);

protected:
	virtual void recv(Packet *, Handler *);
	int command(int argc, const char*const* argv);
        Vaad_BeaconTimer *beacon_timer_;
	Vaad_AdTimer *ad_timer_;
        Packet *makeAlive(void);
	
	Trace *tracetarget;		// Trace Target
	int status_;    	  
    void trace(char *fmt, ...);
    void tracepkt(Packet *, double, int, const char *);

	void process_data_in(const Packet *, int);
	void proc_data_out(const Packet *);
        int  proc_beaconIn(const Packet *, int);
	void send_exp_arq(hdr_vaad *);
	//UC3M - ADDED METHOD
	void send_new_neighbour(const Packet *);
	 NeiEntity *findNextHop(double *, double *, int L);
         NeiEntity *findNextHopUC3M(double *, double *, int L);
	 void recordStop(int , double , int , int );
	 void updateNumReceiver(int );
	 void updateNumRelayer(int );
	
	double data_exp_int_;  //interval for data expiration timer
	double data_con_int_;
	double token_con_int_;

	int L;    //road segment length
	double prob; // filtering probability
        //UC3M - ADDED NEXT LINE
	int longCad; //adchain length
	int ED;       //E{D}
	int Di;      // distance for the current ad transmission
	double *prob_thrs;
	int flag_; //honest or dishonest node
	int type_;  //Source node (1), dest. node (2) or common node (0)
	int base_id_;  //base packet ID. The data packet will have increasing IDs based on this one.
	
	/* statistics */
	int pkt_sent_;
	int pkt_recv_;

	int last_recv_id;
	int last_send_id;
        double bint_;       //Beacon interval
        double bdesync_;    //desynchronized factor.
	 
  int verbose_;			// verbos//	void data_gen_callback(void);

  int drop_debug_;		// whether or not to be verbose on NRTE events  
  LocDbase *ldb_;		// location database
  MobileNode *mn_;		// my MobileNode
  PriQueue *ifq_;		// my ifq
  int off_vaad_;		// offset of the GPSR packet header in pkt
  NsObject *port_dmux_;         // my port demux
  int ID_;    //Node ID (MAC Address) of this node, obtained from the MAC object.
  char *name;
   
  NeiTable * table;
  int nextSeg;
  recordStruct ** record;
  int recordSize ;    
  Ad_Info my_ad;
  int next_hop_;
  int retrans;
  /* functions */
  void init(); 	
};

/* 
Here we define the class for neighbor list maintainance
*/
class NeiEntity
{
  public:
     NeiEntity(): dst(0), x(0), y(0), z(0), latest_time(0x0)
	 { memset(name, 0, 20);}
     ~NeiEntity() 
	 { 
	  ; //nothing at present.
	 }
  
	nsaddr_t dst;  //ip address of this neighbor entity
    double x;     
    double y; 
    double z;     
    double latest_time;   //last updated time
    char name[20];
};

class NeiTable
{
  public:
	NeiTable(double );
	~NeiTable();
	
	NeiEntity **tab;
	int Iter;
	int num_ne;
	int max_ne;
        double Timeout;
	
	void add_ent(NeiEntity *);
	void del_ent(NeiEntity *);
	void reset();	
	int count();
	NeiEntity * Iterate();
	void resetIter();
        void checkTiming(double , double );
};

/* we have three timers here: data_packet_timeout, data_contention, and token_contention . These timers should call the callback function of the VaadAgent */
/* definition of the timers */
class Vaad_AgentTimer: public TimerHandler
{
public:
	Vaad_AgentTimer(VaadAgent *a_) {a=a_;}
	virtual void expire(Event *e) = 0;
protected: 
	VaadAgent *a;
}; 

class Vaad_BeaconTimer: public Vaad_AgentTimer
{
public:
	Vaad_BeaconTimer(VaadAgent *a_):Vaad_AgentTimer(a_){};
	virtual void expire(Event *){a->beacon_callback();}
};

//Vaad_AdTimer
class Vaad_AdTimer: public Vaad_AgentTimer
{
public:
	Vaad_AdTimer(VaadAgent *a_):Vaad_AgentTimer(a_){};
	virtual void expire(Event *){a->ad_callback();}
};
