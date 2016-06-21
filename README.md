PADIV - Privacy-preserving Advertisement
Dissemination with balanced Incentives in VANETs

Experimental case study
--------------------------

VERSION
--------

Version: 2016-06-21


AUTHORS
---------
Original scripts, NS files and driving scenario (from VAAD paper): Zhengming Li (zli1 at mtu.edu)

Adaptations for PADIV in scripts and NS files (marked with UC3M) and Manhattan driving scenario: Jose Maria de Fuentes (jfuentes at inf.uc3m.es), Lorena Gonzalez (lgmanzan at inf.uc3m.es)

PADIV paper: Lorena Gonzalez (lgmanzan at inf.uc3m.es), Jose Maria de Fuentes (jfuentes at inf.uc3m.es), and Jetzabel Serna (Jetzabel.Serna@m-chair.de)

INSTALLATION INSTRUCTIONS
---------------------------

In order to run this script the following elements are prerequisites:
* PERL
* TCL/TK
* NS 2.34 

USAGE
------

mainScript.pl : This script invokes NS with the simulationScript.tcl. It will run with different combinations of the probability to take a coupon, the dissemination distance and the amount of coupons forming a list. 

No parameters are needed. However, you may consider changing the driving scenario in the simulationScript.tcl

In order for this script to run, you must install vaad.cc and vaad.h files in NS, so that the simulated agents (i.e. the driving vehicles) know what they have to do.

COPYRIGHT NOTICES
---------------------
The simulationScript is based on the script provided by Zhengming Li, as the author of VAAD paper. It has been adapted by PADIV authors to meet the new goals pursued by their contribution. All adapted parts have been marked by UC3M, and they are the only elements whose intellectual property belong to PADIV authors.

In the same direction, vaad.cc and vaad.h were provided by Zhengming Li, as the author of VAAD paper. It has been adapted by PADIV authors to meet the new goals pursued by their contribution. All adapted parts have been marked by UC3M, and they are the only elements whose intellectual property belong to PADIV authors. 

Regarding the driving scenarios, vaad_10k_nolight_1800.tcl was provided by Zhengming Li, whereas Urban_Mhnt_750n_1000s_1.tcl was created by PADIV authors.

#THIS SOFTWARE IS PROVIDED BY ITS AUTHORS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
#ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

