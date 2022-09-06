//**************************************************************
// Version dérivée de AEMM ( lui même dérivé de AEPL & AECP & ) Multi Etincelles Nano. Allumage électronique programmable - Arduino
// Différences pour celle-ci 2 capteurs SR17J6 placés à 180° et cible tournatnte de 61°, et attaque directe sur les ports arduino au lieu des "digitalwrite -> " gain de cycle programme"
// Pas d'utilisation de la librairie Timerone.h

char ver[] = "version Seb JACQ 31-12-2021";
// Allumage à 0° d'avance au démarrage au kick (fin de cible au PMH), Temps d'avance non calculé jusqu'à Nplancher
// La cible doit donc couvrir ici : 61° vilebrequin, et pas juste le point de déclenchement  ****** ATTENTION CIBLE 61° ******
// En option, connexion d'un sélecteur de courbe entre la patte A4 et la masse,  //Enlever de la boucle loop, pour gagner du temps de cycle
// Pour changer de courbe d'avance en fonction du régime moteur (courbe d'avance mécanique)
// T > 14ms, correction Christophe.
// LED de calage statique de l'allumage
// Avance fixe jusqu'à Nplancher et temps de charge bobine plus régulier qu'avant
// Temps de charge bobine:
//  -En dessous de Nplancher le temps de charge bobine correspond au passage de la cible devant le capteur (Sécurité mise à 1s, pour préserver les bobines)-> si cible devant capteur pendant plus d'une seconde le courant est coupé)
//  -Entre Nplancher et Nseuil le temps de charge est diminue de Tplancher/360*angleCible quand on est à Nplancher jusqu'a TchargeBob qund on est à Nseuil
//  -Au dessus de Nplancher le temps de charge bobine correspond à Tchargebob
//  -A haut régime si T (1/2 tour moteur) est inférieur à TchargeBob, alors on réduit de temps d'alimentation bobine
//******************************************************************************

// Si on passe à 1 un de ces Debug on peut lire les valeurs lors du déroulement du programme, mais il est ralenti pas ces lectures donc ne pas laisser à 1 pour le fonctionnement normal (sauf printMM)

#define Debug_CalculT 0        // Lire T dans CalculD
#define Debug_CalculD 0        // Lire D dans CalculD
#define Debug_Tloop 0          // Lire T dans Loop
#define Debug_Davant_TchargeBob 0 // Lire temps avant recharge bobine 
#define Debug_printMM 0        // Sortie série N régime et Av avance (peut servir avec PC, ou Smartphone, ou Tablette, pour lire régime moteur et avance)

// Si on ajoute pc(nom_de_la_variable) dans une ligne du programme, il s'arrête en donnant la valeur de cette variable (ex pc(T), pc(D) etc.)  et continue si on met un caractère et "envoyer"
#define pc(v) Serial.print("Ligne_") ; Serial.print(__LINE__) ; Serial.print(" ; ") ; Serial.print(#v) ; Serial.print(" = ") ; Serial.println((v)) ; Serial.println(" un caractère "a" par exemple et 'Envoyer' pour continuer") ; while (Serial.available()==0);{ int k_ = Serial.parseInt() ;}




//************** Ces lignes sont à renseigner obligatoirement.****************
// Ce sont : Na[],Anga[], Ncyl, AngleCapteur, CaptOn, Dwell
// Les tableau Na[] et Anga[] doivent obligatoirement débuter puis se terminer par 0
// et contenir des valeurs entières >= 1
// Le nombre de points est libre. (L'avance est fixe entre Ndem et Nplancher t/mn)
// Le dernier N fixe la ligne rouge, c'est à dire la coupure de l'allumage
//***************  Courbe Avance régime à renseigner

// *********************************************************************************************Courbe d'avance KAWASAKI Z650 1979 *********************************************************************************************
// Au régime moteur de (obligatoirement croissant):
int Na[] = {0, 100, 500,  600,   700,   800,   910,  1000,  1200,  1400,    1500,   2000,      2500,   3500,   4000,    6000, 7000, 8000, 10500,    0}; // Courbe a
          // degrés d'avance vilebrequin correspondant:
int Anga[] = {0,3,   5 ,    6,     7,     14,    11,   10,     10,   12,      14,     17    ,    24,     32,     34,      36,  37,   37,   35,     0  };

int Ncyl = 4;                     // Nombre de cylindres
const int AngleCapteur = 61;      // Position en degrès (vilebrequin) avant le PMH, du capteur (Hall ou autre), coté distribution et allumeur c'est 36° pour toujours avoir AngleCapteur = 72° (vilebrequin)
const int CaptOn = 1;             // CapteurOn = 1 déclenche sur front montant (capteur saturé) le capteur conduit quand il n'y a pas le disque qui fait écran à l'aimant                                                    ///// Seb
// CapteurOn = 0 déclenche sur front descendant (capteur non saturé). le capteur Hall conduit (donc 0 V) quand le disque aimanté passe devant Voir explications en fin du listing

//***** MULTICOURBES **** IL FAUT TOURNER LE SELECTEUR POUR SELECTIONNER LA COURBE D'AVANCE *******
// A la place de la courbe a, on peut sélectionner la courbe b, c, d ou e
// Un sélecteur rotatif et 4 résistances de 4K7, 18K (ou 22k), 47K et 100K font le job
// avec l'entrée configurée en Input Pull-up, ainsi s'il y a une coupure on reste sur la valeur par défaut
//***************  les autres Courbes Avance régime à renseigner si besoin
//*******//*********Courbe   b
int Nb[] = {0, 500, 600, 700,  800,  900, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 6800, 7000, 0};   // Courbe b
int Angb[] = {0, 5,  10,  14,   14,   10,   14,   22,   24,   26,   30,   30,   30,   28,   5,  0};
//*******//*********Courbe   c
int Nc[] = {0, 500, 600, 700,  800,  900, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 6800, 7000, 0};   // Courbe c
int Angc[] = {0, 5,  12,  16,   16,   12,   16,   24,   26,   28,   32,   32,   32,   28,   5,  0};
//*******//*********Courbe   d
int Nd[] = {0, 500, 600, 700,  800,  900, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 6800, 7000, 0};   // Courbe d
int Angd[] = {0, 5,  14,  18,   18,   14,   18,   26,   28,   30,   34,   34,   34,   28,   5,  0};
//*******//*********Courbe   e
int Ne[] = {0, 500, 600, 700,  800,  900, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 6800, 7000, 0};   // Courbe e
int Ange[] = {0, 5,  16,  20,   20,   16,   20,   28,   30,   32,   36,   36,   36,   28,   5,  0};
//**********************************************************************************


// Valable pour tout type de capteur soit sur vilo soit dans l'allumeur (ou sur l'arbre à came)
// La Led(D13) existant sur tout Arduino est allumée lorsque la cile est en face des capteur utile pour caler l'allumage
// En option, multi-étincelles à bas régime pour dénoyer les bougies
// En option,multi courbes d'avance "centrifuge", 5 courbes possibles, selectionables par A4
// Avance 10°jusqu'a Nplancher t/mn

//***************************************************************************************************************************************************************************************
//***************************************************Ces valeurs sont eventuellement modifiables*****************************************************************************************
//***************************************************************************************************************************************************************************************
// Ce sont Nkick Nplancher, TchargeBob , DsecuBob, Ndem,
//const unsigned long TsecuKick = 1000000;      // Sécurite: remettre à 0 N tours moteur si le moteur a calé, temps en uS
//const int NgestBob = 900;                     // Seuil du régime où isr Gest Bobine entre en action 1400 tr/mn maxi sinon plus assez de temps de charge avec 72°
const int Ndem = 600;                          // Vitesse estimée du moteur (ne sert que pour le premier tour moteur, au démarrage ou quand le moteur a calé)
const int Nplancher = 900;                   // vitesse en t/mn jusqu'à laquelle l'avance est fixe à 0°, doit être supérieur à Ndem, au-dessus l'avance est celle programmée et avec correction
const int Nseuil = 3500;                      // Seuil en dessous duquel le temps de charge bobine est majoré de CoeffBob (entre NgestBob et Nseuil)
const unsigned long DsecuBob = 1000000;      // Sécurite: bobine coupée après "DsecuBob" en µs, MM normalement n'entre jamais en action
const int TchargeBob = 3500;                 // temps de recharge bobine nominal pour 14,5V batterie, ex 7 ms bobines Lucas T140, 5 ms à 14,5 V U batterie pour bobine Bosch crayon (mesuré sur Clio 1.6L 16S)        /////////Seb par extrapolation pour atteindre 3000tr/min
const int Multi = 1;         // 1 pour multi-étincelles, 0 pour mettre HS
const int N_multi = 800;    // limite supérieure en t/mn du multi-étincelles


//*****************************************************************************************************************************************************************************************
//*********************** tcor à retoucher en cas de modification du programme, car ce temps est une compensation du temps d'exécution du programme****************************************
//*****************************************************************************************************************************************************************************************
unsigned int tcor = 100;              // correction en µs du temps d'exécution du programme                                                                                               *
//*****************************************************************************************************************************************************************************************

//*****************************************************************************************************************************************************************************************
//********************************************** Constantes du sketch *********************************************************************************************************************
//*****************************************************************************************************************************************************************************************
const byte Cible14_On = B00000100;          // Mise à 0 du bit N°5 pour mettre à HIGH sortie D13 (PORTB du NANO)
const byte Cible14_Off = B00000000;
const byte Cible23_On = B00001000;
const byte Cible23_Off = B00000000;
const int SelecAv = A4;              // Entrée analogique sur A4 pour sélecteur de changement de courbes avance centrifuge. R PullUp
const byte Bobbine_1_HIGH = B00010000;     // Mise à 1 du bit N°4 pour mettre à HIGH sortie D4 (PORTD du NANO)
const byte Bobbine_1_LOW =  B11101111;     // Mise à 0 du bit N°4 pour mettre à LOW sortie D4  (PORTD du NANO)
const byte Bobbine_2_LOW =  B11111110;     // Mise à 0 du bit N°0 pour mettre à LOW sortie D8 (PORTB du NANO)
const byte Bobbine_2_HIGH = B00000001;     // Mise à 1 du bit N°0 pour mettre à HIGH sortie D8 (PORTB du NANO)
const byte Led13_HIGH = B00100000;         // Mise à 1 du bit N°5 pour mettre à HIGH sortie D13 (PORTB du NANO)
const byte Led13_LOW = B11011111;          // Mise à 0 du bit N°5 pour mettre à HIGH sortie D13 (PORTB du NANO)

//*******************************************************************************************
const int Cible14 = 2;                // Entrée sur D2 du capteur, R PullUp et interrupt
const int Cible23 = 3;                // Entrée sur D3 du capteur, R PullUp et interrupt
float modC1 = 0;                    // MM Correctif pour C1[]
unsigned long D = 0;                // D délai en µs et ms à attendre après la cible pour l'étincelle
unsigned long Dsum = 0;             // D total
int valSelecAv = 0;                 // 0 à 1023 selon la position du sélecteur en entrée de courbe d'avance "centrifuge"
int milli_delay = 0;
int micro_delay = 0;
float Tplancher = 0;                // Seuil pour régime plancher
unsigned long T_chargeBob_plancher = 0;
unsigned long Davant_TchargeBob = 0;// Délai en µs après étincelle pour démarrer la recharge bobine.
unsigned long Tprec = 0;            // Période précédant la T en cours, pour calcul de TchargeBob
unsigned long prec_H = 0;           // Heure du front précédent en µs
unsigned long T = 0;                // Période en cours
unsigned long Tdem = 0;             // Valeur de T au démarrage pour le premier tour moteur, exemple 200 ms pour 150 tr/mn
unsigned short DsecuBob_timer = 0;  // Valeur DsecuBob dans timer1
int N1 = 0;                         // Couple N,A de début d'un segment
int Ang1 = 0;                       // Car A1 réservé pour entrée analogique!
int N2 = 0;                         // Couple N,A de fin de segment
int Ang2 = 0;
int*  pN = &Na[0];                  // pointeur au tableau des régimes. Na sera la courbe par défaut
int*  pA = &Anga[0];                // pointeur au tableau des avances. Anga sera la courbe par défaut
float k = 0;                        // Constante pour calcul de l'avance courante
float C1[20];                       // Tableaux des constantes de calcul de l'avance courante
float C2[20];                       // Tableaux des constantes de calcul de l'avance courante
float Tc[20];                       // Tableau des Ti correspondants au Ni
// Si nécessaire, augmenter ces 3 valeurs: Ex C1[30],C2[30],Tc[30]
int Tlim  = 0;                      // Période minimale, limite, pour la ligne rouge
int j_lim = 0;                      // index maxi des N, donc aussi Ang
unsigned long NT  = 0;              // Facteur de conversion entre N et T à Ncyl donné
int AngleCibles = 0;                // Angle entre 2 cibles, 180° pour 4 cyl, 120° pour 6 cyl, par exemple
int UneEtin = 1;                    // = 1 pour chaque étincelle, testé par isr_CoupeI et remis à zero
boolean Memoire_cible14 = false;              // Mémorisation du capteur 1-4 à on pour etincelle immédiate
boolean Memoire_cible23 = false;             // Mémorisation du capteur 2-3 à on pour etincelle immédiate
int Cible_detectee = 0;
unsigned long T_multi  = 0;         // Période minimale pour multi-étincelles
unsigned long Tseuil = 0;               //Seuil en dessous duquel le temps de charge bobine est majoré de 50%
float Vitesse = 0;              // vitesse en cours
unsigned int Regime = 0;        // régime moteur affiché (partie entière de vitesse)
byte N = 0;                     // MM comptage des tours moteurs pour le démarrage au kick
unsigned long Delaideg = 0;     // µs/deg pour la dépression
unsigned long Stop_temps_Etincelle = 0;    // MM ajout de = 0
unsigned long Tempsecoule = 0;
float AvanceMoteur = 0;
int fxAvanceMoteur = 0;
unsigned int fxAvanceMoteur2 = 0;

///////////////variable ralenti stabilisé///////////
//*************RÃ©gulation du ralenti par modification dynamique de l'avance
//* Sur une idÃ©e d'Antoine Zorgati
//* L'avance au ralenti est totalement indÃ©pendante de la courbe   d'allumage
//* On fixe une avance de dÃ©part AvR = 10 degrÃ¨s et une zone de consigne de ralenti avec une tolÃ©rance, par exemple 800+-50tr/min
//* Si le rÃ©gime est hors consigne, on corrige l'avance de 1 degrÃ¨ en plus ou en moins.
//* On conserve cette avance pendant un nombre donnÃ© CtCyInit de cycles moteurs pour stabiliser le rÃ©gime
//* Puis on vÃ©rifie de nouveau le rÃ©gime: si Ã  l'intÃ©rieur de la consigne, on ne touche pas Ã  l'avance, si hors consigne on la corrige d'un point
//* Ainsi de suite dans la zone de ralenti dÃ©finie par NrMin et NrMax...

const int  Nr = 1100; //Consigne de ralenti rÃ©gulÃ© en t/mn,800 par exemple. Si 0 pas de regulation.
int NrMin = 1000; //la zone de ralenti est de NrMin Ã  NrMax
int NrMax = 1300;
int unsigned long TrMax = 0; //Pour s'ejecter du ralenti  rapidement
int Nrinf = 1050; //La zone de regulation est de Nrinf Ã  Nrsup
int Nrsup = 1150;
int CtCy = 0; //Compteur de cycles/periodes entre deux mesures de ralenti rÃ©gulÃ©
int CtCyInit = 5;//Soit CtCyInit/2 tours moteurs sur un 4 cylindres
int AvR = 0; //Avance ajustÃ©e dynamiquement pour le ralenti rÃ©gulÃ©
int AvRMin = 9; //Bornes pour l'avance au ralenti rÃ©gulÃ©
int AvRMax = 15; //20 ou 25 ou 30 degrÃ¨s va aussi
int N_ralenti = 0; //En t/mn pour tests de ralenti rÃ©gulÃ©
///////////////////////////////////////////////////////////////////

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

//********************************************************************************************************************************************************************************************************************
//****************************************************************************************************** INITIALISATION **********************************************************************************************
//********************************************************************************************************************************************************************************************************************


void setup()                
{
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  DDRD = B00010000; // active le port D, N°4 en sortie        -> sortie arduino N°D4
  DDRB = B00100001; // active le port B, N°0 et 5 en sortie   -> sortie arduino N°D8 et 13
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(Cible14, INPUT_PULLUP);    // Entrée interruption sur D2, front descendant (résistance pullup)                     //INPUT_PULLUP
  pinMode(Cible23, INPUT_PULLUP);    // Entrée interruption sur D3, front descendant (résistance pullup)              //INPUT_PULLUP

  Serial.begin(115200);         // Ligne suivante, 3 Macros du langage C
  Serial.println(__FILE__); Serial.println(__DATE__); Serial.println(__TIME__);
  Serial.println(ver);

  if (Ncyl < 2)Ncyl = 2;           // On assimile le mono cylindre au bi, avec une étincelle perdue
  pinMode(SelecAv, INPUT_PULLUP);  // Entrée sélecteur rotatif avec résistances, optionnel ! Sélection courbe d'avance
  // set up the ADC
  ADCSRA &= ~PS_128;            // remove bits set by Arduino library
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_64;
  // set our own prescaler to 64

  Init();                       // Executée une fois au démarrage et à chaque changement de courbe
}


void  Init ()           //////////////////// Calcul des tableaux d'avance /////////////////////////////
// Calcul de 3 tableaux,C1,C2 et Tc qui serviront à calculer D, temps d'attente
// entre la détection d'une cible par le capteur  et la génération de l'étincelle.
// Le couple C1,C2 est determiné par la période T entre deux cibles, correspondant au
// bon segment de la courbe d'avance entrée par l'utilisateur: T est comparée à Tc
{
  AngleCibles = 720 / Ncyl;                    // Ex : 720° pour 1 ou 2 cylindres (à étincelle perdue), 180° pour 4 cylindres, et 120° pour 6 cylindres
  NT = 120000000 / Ncyl;                       // Facteur de conversion Nt/mn en T µs
  T_multi = NT / N_multi;                      // Période minimale pour génerer un train d'étincelles
  Tdem = NT / Ndem;                            // Période de la première étincelle
  Tplancher = NT / Nplancher;                  // T à vitesse plancher en t/mn, en dessous, avance centrifuge = 0
  T_chargeBob_plancher =  Tplancher * 2 * AngleCapteur / 360; //Calcul du temps de charge bobine à la sortie de dwell fixe (quand T<Tplancher) pour que le dwell soit dégressif jusqu'à N seuil
  Tseuil = NT / Nseuil;                        // T à vitesse seuil en t/mn, entre NgestBob et Nseuil majoration de la charge bobine de 50%
  DsecuBob_timer = DsecuBob * 8 / 1024; //calcul de DsecuBob pour timer 1 avec prescaler à 1024

  TrMax = NT / NrMax; //Pour ralenti rÃ©gulÃ©

  Select_Courbe_Avance();                      // Ajuster éventuellement les pointeurs pN et pA pour la courbe a, b, c, d ou e

  N1  = 0; Ang1 = 0;                           // Toute courbe part de  0
  int i = 0;                                   // locale mais valable hors du FOR
  pN++; pA++;                                  // sauter le premier élément de tableau, toujours =0
  for (i  = 1; *pN != 0; i++)                  // i pour les C1,C2 et Tc. Arrêt quand régime=0.
    // pN est une adresse (pointeur) qui pointe au tableau N. Le contenu pointé est *pN
  { N2 = *pN; Ang2 = *pA;                      // recopier les valeurs pointées dans N2 et Ang2
    k = float(Ang2 - Ang1) / float(N2  - N1);
    C1[i] = float(AngleCapteur - Ang1 + k * N1) / float(AngleCibles);  // MM suppression Avancestatique
    C2[i] = -  float(NT * k) / float(AngleCibles);
    Tc[i] = float(NT / N2);
    N1 = N2; Ang1 = Ang2;                      // fin de ce segment, début du suivant
    pN++; pA++;                                // Pointer à l'élément suivant de chaque tableau
  }
  j_lim = i - 1;                               // Revenir au dernier couple entré
  Tlim  = Tc[j_lim];                           // Ligne rouge

  // Timer1 a deux roles:
  // 1....couper le courant dans la bobine en l'absence d'étincelle pendant plus de DsecuBob µs
  // 2... après une étincelle, attendre le délai D recharge bobine avant de rétablir le courant dans la bobine
  // Ce courant n'est rétabli que TchargeBob ms avant la prochaine étincelle, condition indispensable
  // pour une bobine à faible résistance, disons inférieure à 3 ohms. Typiquement TchargeBob = 3ms.

  //*******************************************************************************************************
  //Configuration timer 1
  TCCR1A = 0;                                   // clear control register A
  TIMSK1 = _BV(TOIE1);                          //attachInterrupt ISR(TIMER1_OVF_vect)
  //*********************************************************************************************************

  PORTD &= Bobbine_1_LOW;                       // par principe, couper la                                                                   bob1=0
  delayMicroseconds (1);
  PORTB &= Bobbine_2_LOW;                       // par principe, couper la                                                                   bob2=0
  delayMicroseconds (1);
}

//***********************************************************************************************************************************************************************************************************
//*****************************************************************************************************LES FONCTIONS*****************************************************************************************
//***********************************************************************************************************************************************************************************************************
void  CalcD ()    //*********************************** Calcul du temps D entre top capteur à 60° et étincelle d'allumage************************************************************
{
  //*************************************************************
  //Si la bobine n'est pas alimentée (le cas lors du premier passage de l'alim Ngestbob à calcD)
  if (UneEtin > 0 && Tprec >= Tplancher)
  {
    AlimBobine();
  }

  // Noter que T1>T2>T3...
  for (byte j = 1; j <= j_lim; j++)                                                    // On commence par T la plus longue et on remonte, tant que j < limite on augmente j + 1
  { // Serial.print("Tc = "); Serial.println(Tc[j]);delay(1000);

    ///////////////////////// Debug_CalculT //////////////////
    //  if ( Debug_CalculT) {
    //       Serial.print(" ; T = ");  Serial.println(T);
    //   }
    ////////////////////////////////////////////////////////

    if  (T >=  Tc[j]) {                                                               // Si T période en cours supérieur ou égal à j tableau des Ti et Ni
      // on a trouvé le bon segment de la courbe d'avance

      D =  float(T * ( C1[j] - modC1 ) + C2[j]) ;   // MM D en µs = T x (C1 - correctif C1) + C2

      ///////////////////////// Debug_CalculD //////////////////
      if ( Debug_CalculD) {
        Serial.print(" ; D = ");  Serial.println(D);
      }
      ////////////////////////////////////////////////////////

      Dsum = D - tcor ;                                                    // tcor appliqué ici donne de meilleurs résultats niveau précision

      if (Dsum < 6500) {                                                  // delay maxi à ne pas dépasser 16383 µs, risque de temps totalement incorrect
        //Dsum= (Dsum * 3) - 8;                                                                                                          ///////////////////////////////// Seb correction de Dsum suite à essai sur moniteur série
        delayMicroseconds ((Dsum));                                       // Attendre Dsum
      }
      else {
        milli_delay = ((Dsum / 1000) - 2);                                // Quand D > 10ms car problèmes avec delayMicroseconds(D) si D>14ms!  fwb
        micro_delay = (Dsum - (milli_delay * 1000));
        delay(milli_delay);                                               // Currently, the largest value that will produce an accurate delay is 16383 µs
        delayMicroseconds(micro_delay);
      }
      break;                                                              // break;  //Sortir, on a D
    }
  }
}

void  CalcD_Test()   ///////////////////////////////////////
{
  if (T < TrMax)CalcD(); //Hors ralenti
  else
  {
    if (Nr == 0)CalcD(); //En ralenti mais pas de regul
    else
      RegRalenti();
  }
}

void  RegRalenti()////////////////////////////////////
{ //On va calculer un D specifique pour le ralenti rÃ©gulÃ©, ignorant les courbes d'avance
  CtCy++;  //Compteur de passage, on ne teste que tous les CtCyInit passages
  if (CtCy >= CtCyInit)  //Sinon,conserver AvR pour calculer D
  { //Verifier AvR
    CtCy = 0; //Re init le compteur de passages
    N_ralenti = NT / T; //Calculer N pour les tests Ã  suivre
    if (N_ralenti < Nrinf || N_ralenti > Nrsup)  //Sinon c'est bon  pas de correction, garder AvR
    { //correction necessaire pour AvR
      if (N_ralenti < Nrinf)
      {
        AvR++; //Trop lent
        if (AvR > AvRMax)AvR = AvRMax; //Plafonner haut
      }
      else
      { AvR--; //Trop vite
        if (AvR < AvRMin)AvR = AvRMin; //Plafonner bas
      }
    }
  }
  D = (T / AngleCibles) * (AngleCapteur - AvR);//On neglige les corrections car au ralenti
        ////////////////////////////////////////////////////////

      Dsum = D - tcor ;                                                    // tcor appliqué ici donne de meilleurs résultats niveau précision

      if (Dsum < 6500) {                                                  // delay maxi à ne pas dépasser 16383 µs, risque de temps totalement incorrect
        //Dsum= (Dsum * 3) - 8;                                                                                                          ///////////////////////////////// Seb correction de Dsum suite à essai sur moniteur série
        delayMicroseconds ((Dsum));                                       // Attendre Dsum
      }
      else {
        milli_delay = ((Dsum / 1000) - 2);                                // Quand D > 10ms car problèmes avec delayMicroseconds(D) si D>14ms!  fwb
        micro_delay = (Dsum - (milli_delay * 1000));
        delay(milli_delay);                                               // Currently, the largest value that will produce an accurate delay is 16383 µs
        delayMicroseconds(micro_delay);
      }
}


void  Etincelle ()                                    //*********** Coupure de la bobine pour créer l'étincelle d'allumage******************

{ // Etincelle bobine 1-4 si capteur 1-4 à été activé
  if (Memoire_cible14 == true) {                                        //si capteur 1-4 actif, alors allumage sur bobine 1-4
    PORTD &= Bobbine_1_LOW;
    delayMicroseconds (20);
    UneEtin = 1;                                                        // Signaler une étincelle bobine 1-4 à l'isr_GestionIbob().
  }
  // Etincelle bobine 2-3 si capteur 2-3 à été activé
  if (Memoire_cible23 == true) {                                       //Si capteur 2-3 actif, alors allumage sur bobine 1-4
    PORTB &= Bobbine_2_LOW;                                            // Couper le courant, donc étincelle
    delayMicroseconds (20);
    UneEtin = 2;                                                      // Signaler une étincelle bobine 2-3 à l'isr_GestionIbob().
  }
  Stop_temps_Etincelle = micros();                                    // MM Calcul des temps
  TIMSK1 &= ~_BV(TOIE1);                                              // détacher interruption


  if (Multi && (T >= T_multi))
  {
    Genere_multi();                                                   // Si on a multi étincelles et T supérieur à temps multi on génère multi éticelles
  }
  else {
    if (T <= Tplancher) {
      if (T <= Tplancher && T >= Tseuil)                                     // MM le seuil est mesuré à l'oscilloscope et calculé en fonction du Temps de charge Bobine programmé
      {
        Davant_TchargeBob =  T - (map (T , Tplancher, Tseuil, T_chargeBob_plancher , TchargeBob ));           // MM Temps avant recharge de la bobine correction - 23 % entre Ndem et environ 850 tr/mn
      }
      else                                                              // MM sinon calcul sans correction pour les autres régimes moteur, le temps est correct
        Davant_TchargeBob = 2 * T - Tprec - TchargeBob;                 // On doit tenir compte des variations de régime moteur
      Tprec = T;                                                        // Maj de la future periode precedente
      if ( T <= TchargeBob + 300)                                       ///Réduit la durée d'alim bobine si T (temps pour un tour moteur) est inférieur à 2 fois la duréee d'alim bobine
      {
        //   Davant_TchargeBob = 2 * T - Tprec -(T/2);                  //Permet de ne pas bloquer le programme si durée alim bobine est supérieure à 1/2 tour moteur
        Davant_TchargeBob = 1.1 * T - Tprec;
      }

      // Attendre Davant_TchargeBob (temps de charge bobine) en µs avant de rétablir le courant dans la bobine
      //**********************************************************************************************************************
      TCCR1B = 0;                           // set mode as phase and frequency correct pwm, stop the timer
      TCCR1A = 0;                                   // clear control register A
      ICR1 = Davant_TchargeBob;                   // 8=(F_CPU / 2000000)
      TCCR1B = _BV(WGM13) | _BV(CS11) ;//| _BV(CS10); //prescaler sur 8 -> valeur maxi 32ms
      TIMSK1 = _BV(TOIE1);
      //*********************************************************************************************************************
    }
  }
  ///////////////////////// Debug_Davant_TchargeBob //////////////////
  if ( Debug_Davant_TchargeBob) {
    Serial.print(" ; Davant_TchargeBob = ");  Serial.println(Davant_TchargeBob);
  }
  ////////////////////////////////////////////////////////


}




void  Genere_multi()                       ////////// Gérération de multi étincelles après allumage normal
{
  if (Memoire_cible14 == true)                              //Seb si capteur 1-4 actif, alors allumage sur bobine 1-4
  { // L'étincelle principale a juste été générée
    delayMicroseconds(1000);                                // Attendre fin d'étincelle T en µs
    PORTD |= Bobbine_1_HIGH;                                // Rétablir le courant                                          Bob1=1
    delayMicroseconds((TchargeBob) * 0.2);                  // Recharger en : "TchargeBob" x coefficient, en µs
    PORTD &= Bobbine_1_LOW;                                 // Première étincelle secondaire                                Bob1=0
    delayMicroseconds(200);                                 // Attendre fin d'étincelle T en µs
    PORTD |= Bobbine_1_HIGH;                                // Rétablir le courant                                          Bob1=1
    delayMicroseconds((TchargeBob) * 0.2);                  // Recharger en : "TchargeBob" x coefficient, en µs
    PORTD &= Bobbine_1_LOW;                                 // Deuxième étincelle secondaire                                Bob1=0
    delayMicroseconds(200);                                 // Attendre fin d'étincelle T en µs
    PORTD |= Bobbine_1_HIGH;                                // Rétablir le courant                                          Bob1=1
    delayMicroseconds((TchargeBob) * 0.2);                  // Recharger en : "TchargeBob" x coefficient, en µs
    PORTD &= Bobbine_1_LOW;                                 // Troisième étincelle secondaire                               Bob1=0
    delayMicroseconds (1);
  }
  if (Memoire_cible23 == true)
  { // L'étincelle principale a juste été générée
    delayMicroseconds(1000);                                 // Attendre fin d'étincelle T en µs
    PORTB |= Bobbine_2_HIGH;                                // Rétablir le courant                                          Bob2=1
    delayMicroseconds((TchargeBob) * 0.2);                  // Recharger en : "TchargeBob" x coefficient, en µs
    PORTB &= Bobbine_2_LOW;                                 // Première étincelle secondaire                                Bob2=0
    delayMicroseconds(200);                                 // Attendre fin d'étincelle T en µs
    PORTB |= Bobbine_2_HIGH;                                // Rétablir le courant                                          Bob2=1
    delayMicroseconds((TchargeBob) * 0.2);                  // Recharger en : "TchargeBob" x coefficient, en µs
    PORTB &= Bobbine_2_LOW;                                 // Deuxième étincelle secondaire                                Bob2=0
    delayMicroseconds(200);                                 // Attendre fin d'étincelle T en µs
    PORTB |= Bobbine_2_HIGH;                                // Rétablir le courant                                          Bob2=1
    delayMicroseconds((TchargeBob) * 0.2);                  // Recharger en : "TchargeBob" x coefficient, en µs
    PORTB &= Bobbine_2_LOW;                                 // Troisième étincelle secondaire                               Bob2=0
    delayMicroseconds (1);
  }
}


ISR(TIMER1_OVF_vect)         ////////// Gestion du moment d'alimentation de la bobine
{
  TCCR1B = _BV(WGM13);                                     // Stpper le timer
  if (UneEtin > 0 && T > Tlim )                            // Si il y a eu une étincelle (fonction Etincelle) MM essai: ET régime moteur inférieur à régime maxi
  {
    AlimBobine();
  }
  else                                                     // Sinon
  {
    PORTD &= Bobbine_1_LOW;                               // Préserver la bobine, couper le courant
    delayMicroseconds (1);
    PORTB &= Bobbine_2_LOW;                               // Préserver la bobine, couper le courant
    delayMicroseconds (1);
  }
}


void AlimBobine()
{
  if (UneEtin == 2) {                                   //Seb si étincelle précédente sur bobine 2-3, alors recharger 1-4
    PORTD |= Bobbine_1_HIGH;                            // Rétablir le courant dans bobine 1-4
    delayMicroseconds (1);
  }
  if (UneEtin == 1) {                                  //Seb si étincelle précédente sur bobine 1-4, alors recharger 2-3
    PORTB |= Bobbine_2_HIGH;                           // Rétablir le courant dans bobine 2-3
    delayMicroseconds (1);
  }

  UneEtin = 0;                                         // Remet le détecteur d'étincelle à 0
  //******************************************************************************************************************************
  // Au cas où le moteur s'arrête, couper l'alimentation de la bobine, après DsecuBob µs, pour ne pas la surchauffer**************
  //******************************************************************************************************************************
  TCCR1B = 0;        // set mode as phase and frequency correct pwm, stop the timer
  TCCR1A = 0;                 // clear control register A
  ICR1 = DsecuBob_timer;  ///0,0078125=(F_CPU / 2000000)/1024*DsecuBob
  TCCR1B = _BV(WGM13) | _BV(CS12) | _BV(CS10); //prescaler sur 1024 -> valeur maxi 8s
}


void  Select_Courbe_Avance()                          // Sélection courbe d'avance "centrifuge" en début de programme
{ // Par défaut, la courbe "a" est déja sélectionnée
  valSelecAv = analogRead(SelecAv);
  Serial.print("Selecteur AV = "); Serial.print(valSelecAv); Serial.print(" ,Centrifuge = ");
  if (valSelecAv < 80 || valSelecAv > 880) {          // Shunt 0 ohm donne 15 ou pas de shunt ou pas de résistance donne 1015
    Serial.println("Courbe a");
  }
  if (valSelecAv > 80 && valSelecAv < 230) {          // Résistance de 4K7 donne 130
    pN = &Nb[0];                                      // pointer à la courbe b
    pA = &Angb[0];
    Serial.println("Courbe b");
  }
  if (valSelecAv > 230 && valSelecAv < 450) {        // Résistance de 18K (ou 22k) donne 340
    pN = &Nc[0];                                     // pointer à la courbe c
    pA = &Angc[0];
    Serial.println("Courbe c");
  }
  if (valSelecAv > 450 && valSelecAv < 650) {       // Résistance de 47K donne 565
    pN = &Nd[0];                                    // pointer à la courbe d
    pA = &Angd[0];
    Serial.println("Courbe d");
  }
  if (valSelecAv > 650 && valSelecAv < 880) {      // Résistance de 100K donne 735
    pN = &Ne[0];                                   // pointer à la courbe e
    pA = &Ange[0];
    Serial.println("Courbe e");
  }
}



void loop()
//***********************************************************************************************************************************************************************************************************
//***************************************************************** Attendre les tops capteur pour générer l'allumage****************************************************************************************
//***********************************************************************************************************************************************************************************************************
{
  while (digitalRead (Cible14) == !CaptOn && digitalRead(Cible23) == !CaptOn);        // Attendre début de la cible du capteur 1-4 ou 3-2                                         //Seb(Mettre || à la place && si Capton à 1 et inversement)
  {
    T = micros() - prec_H;                         // front actif arrivé calculer T
    prec_H = micros();                             // heure du front actuel qui deviendra le front précédent
  }

  //**********************************************************************************************************************************************************************************************************
  //**************************************Permet de garder en mémoire le capteur qui a été activé, pour savoir sur quelle bobine il y'a une étincelle à faire*************************************************
  //**********************************************************************************************************************************************************************************************************
  if (digitalRead(Cible14) == CaptOn)
  {
    Memoire_cible14 = true;
    Memoire_cible23 = false;
  }
  if (digitalRead(Cible23) == CaptOn)
  {
    Memoire_cible23 = true;
    Memoire_cible14 = false;
  }
  //********************************************************************************************************************************************************************************************************
  //******************************************************************Allumage led interne arduino**********************************************************************************************************
  //********************************************************************************************************************************************************************************************************
  PORTB |= B00100000;                                     // MM allumage LED13 de calage statique, quand la cible en métal est présente la LED s'allume
  delayMicroseconds (1);


  //*********************************************************************************************************************************************************************************************************
  //****************************************************************************Gestion au dessus de Nplancher***********************************************************************************************
  //*********************************************************************************************************************************************************************************************************
  if (T <= Tplancher && T > Tlim )           // Si le régime moteur est au dessus du régime de démarrage électrique et sous le régime maxi (zone rouge) et N supérieur au seuil Nkick
  {
    CalcD_Test();                                            // calcul du temps d'attente entre début de la cible et l'allumage
    Etincelle();                                        // Etincelle d'allumage
  }
  else
  {
    //********************************************************************************************************************************************************************************************************
    ///************************************************************************Si moteur à calé***************************************************************************************************************
    //********************************************************************************************************************************************************************************************************
    //Pour T en dessous de DsecuBob, alim bobine des le début de cible
    if (T > DsecuBob)
    {
      N = 0;                                             // MM si T est supérieur à Temporisation sécurité bobine le moteur a calé il faut remettre N à 0 pour un nouveau démarrage
      Tprec = Tdem;
    }
    //********************************************************************************************************************************************************************************************************
    ///************************************************************************Bas régime sous Nplancher******************************************************************************************************
    //********************************************************************************************************************************************************************************************************
    if (T > Tplancher)                                 // MM Si le régime moteur (précédent) est sous le régime N_multi (1500tr/mn) OU N inférieur au seuil Nkick, alimentation bobines pour préparer un allumage
    {
      if (Memoire_cible14 == true) {
        UneEtin = 2;                                   //SJ Permet d'alimenter la bobine au premier tour moteur
      }
      if (Memoire_cible23 == true) {
        UneEtin = 1;                                  //SJ Permet d'alimenter la bobine au premier tour moteur
      }
      AlimBobine();                                   // Alimentation des bobines

      ///////////////////////// Debug_Tloop //////////////////
      //   if ( Debug_Tloop) {
      //       Serial.print(" ; Tloop = ");  Serial.println(T);
      //   }
      ////////////////////////////////////////////////////////
    }
  }
  //***************************************************************************************************************************************************************************************************
  //*********************************************************************Fin de cible allumage*********************************************************************************************************
  //***************************************************************************************************************************************************************************************************
  Tprec = T;                                            // Sauve la valeur de T en cas de perte d'info du capteur
  //Attendre fin de cible
  while (digitalRead(Cible14) == CaptOn || digitalRead(Cible23) == CaptOn);
  //****************************************************************************************************************************************************************************************************
  ///************************************************************************Gestion rupteur************************************************************************************************************
  //****************************************************************************************************************************************************************************************************
  if (Tprec <= Tlim)                                    // Si le régime moteur est égal ou supérieur au régime maxi (zone rouge)
  {
    PORTD &= Bobbine_1_LOW;                            // Préserver la bobine, couper le courant immédiatement au régime maxi sans attendre Dsecu de 1 sec    Bob1=0
    delayMicroseconds (1);
    PORTB &= Bobbine_2_LOW;                            // Préserver la bobine, couper le courant immédiatement au régime maxi sans attendre Dsecu de 1 sec    Bob2=0
    delayMicroseconds (1);
  }


  if (Tprec > Tplancher )                               // MM Si le régime moteur est sous le régime plancher OU N inférieur au seuil Nkick
  {
    Etincelle();                                       // Etincelle d'allumage (et multi-étincelles si à 1) pour le démarrage
  }

  //***************************************************************************************************************************************************************************************************
  //***************************************************************Extinction led interne arduino******************************************************************************************************
  //***************************************************************************************************************************************************************************************************
  PORTB &= B11011111;                                 // Extinction Led13 de calage statique
  delayMicroseconds (1);

  //***************************************************************************************************************************************************************************************************
  //*****************************************************************Passage à 0 des mémoires cible***************************************************************************************************
  //***************************************************************************************************************************************************************************************************
  if (digitalRead(Cible14) != CaptOn) {
    Memoire_cible14 = false;
  }                                                                                                                            //Seb
  if (digitalRead(Cible23) != CaptOn) {
    Memoire_cible23 = false;
  }                                                                                                                            //Seb

  //*****************************************************************************************************************************************************************************************************
  //********************************* MM Informations envoyées vers le port série pour lecture du régime moteur, de l'avance à l'allumage.***************************************************************
  //***************************************************************************** Debug_printMM *********************************************************************************************************
  //*****************************************************************************************************************************************************************************************************
  if (Debug_printMM)
  {
    Vitesse = (NT / T) * 1.001;                                         // MM calcul de la vitesse (régime moteur), avec correction d'erreur 0.001
    Regime = Vitesse;                                                  // MM régime moteur affiché sans chiffres après la virgule
    Delaideg = NT / Vitesse / float(AngleCibles);                     // MM calcul du délai en degrés

    float Tempsecoule = Stop_temps_Etincelle - prec_H;                // MM calcul du temps entre étincelle et le top capteur, type float pour avoir chiffre après la virgule et afficher avance négative sous Ndem
    AvanceMoteur = (AngleCapteur - (Tempsecoule / Delaideg)) * 1.01;  // MM calcul avance totale à l'allumage du moteur, avance moteur doit être du type float, correction 1.01
    fxAvanceMoteur = 10 * AvanceMoteur;                               // MM pour afficher avance avec un chiffre après virgule fixe, optimisation par rapport à virgule flottante
    if (fxAvanceMoteur > 0)
    {
      fxAvanceMoteur2 = fxAvanceMoteur % 10;
    }
    else {
      fxAvanceMoteur2 = -(fxAvanceMoteur % 10);
    }
    Serial.print(" ; N = ");         // MM régime moteur N
    Serial.print(Regime);            // MM régime moteur N
    Serial.print(" ; Av =  ");       // MM avance
    if (AvanceMoteur < -360 || AvanceMoteur > 360)
    {
      Serial.println("All HS"); // MM si l'avance est hors limite afficher : All HS
    }
    else {
      Serial.print(fxAvanceMoteur / 10); // MM partie entière de l'avance
      Serial.print(",");              // MM virgule fixe
      Serial.println(fxAvanceMoteur2);// MM modulo partie après la virgule fixe
    }
  }
}

//******************************************************************************************************************************************************************************************************************
//**********************************************************************Informations diverses **********************************************************************************************************************
//******************************************************************************************************************************************************************************************************************

// Régimes moteur et fréquences correspondantes
// Hertz = Nt/mn / 30 , pour moteur 4 cylindres
// N 1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,8000,8500,9000,9500,10000
// Hz 33,  50,  66,  83  100  117  133  150  166  183  200  216  233  250  266  283  300  316   333

// Hertz = Nt/mn / 60 , pour moteur 2 cylindres
// N 1000,1500,2000,2500,3000,3500,4000,4500,5000,5500,6000,6500,7000,7500,8000,8500,9000,9500,10000
// Hz 16,  25,  33,  41   50   58   66   75   83   91  100  108  116  125  133  141  150  158   166

// Type de capteur Hall possible :

// Capteur Honeywell 1GT101DC, contient un aimant sur le coté, type non saturé, sortie haute à vide,
// et basse avec une cible en métal. Il faut  CapteurOn = 0, déclenche sur front descendant.

// Le capteur à fourche SR 17-J6 contient un aimant en face, type saturé, sortie basse à vide,
// et haute avec une cible métallique. Il faut  CapteurOn = 1, déclenche sur front montant.
