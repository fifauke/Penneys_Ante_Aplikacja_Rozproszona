#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>

#define MAX_CLIENTS 10
#define MAX_GAMES   1000

#define START_ACK_TIMEOUT 3         // w sekundach
#define TOSS_ACK_TIMEOUT 1

#define MSGT_ACK       0
#define MSGT_REGISTER  1
#define MSGT_START     2
#define MSGT_TOSS      3
#define MSGT_REPORT    4
#define MSGT_ENDGAME   5

#define FLAG_ACK_REQUEST  (1 << 0)
#define FLAG_ACK_RESPONSE (1 << 1)

#pragma pack(push, 1)
typedef struct {
    uint8_t  ver_type;      // 4 bity wersja (np. 1), 4 bity typ
    uint8_t  flags;         // bity: ACK_REQUEST, ACK_RESPONSE
    uint16_t seqNum;        // nr sekwencyjny
    uint16_t clientID;      // ID klienta (0 jeœli nieznany albo broadcast)
    uint16_t payloadLength; // d³ugoœæ payloadu
    // za tym polem w buforze payload
} ALP_Header;
#pragma pack(pop)

typedef struct {
    int active;
    uint16_t clientID;
    struct sockaddr_in addr;
    socklen_t addrLen;
    int wins; 
    uint8_t patternByte;
    uint8_t patternLen;
    long totalTossesForWins;
} ClientInfo;

typedef struct {
    int gameID;
    int winnerClientID;
    int winningTossCount;    
} GameRecord;

static ClientInfo g_clients[MAX_CLIENTS];
static int g_numClients = 0;

static GameRecord g_gameRecords[MAX_GAMES];
static int g_currentGameIndex = 0;

static int g_serverSocket = -1;
static struct sockaddr_in g_serverAddr;

static uint16_t g_nextClientID = 1; 
static uint16_t g_seqNum = 1;       // numer sekwencyjny ALP

// statystyki transmisji:
static long g_totalSentPackets = 0;
static long g_totalRecvPackets = 0;
static long g_totalSentBytes = 0;
static long g_totalRecvBytes = 0;

static inline uint8_t alp_getVersion(ALP_Header* hdr) {         
    return (hdr->ver_type >> 4) & 0x0F; 
}
static inline uint8_t alp_getMsgType(ALP_Header* hdr) {
    return (hdr->ver_type & 0x0F);
}
static inline void alp_setVersionType(ALP_Header* hdr, uint8_t version, uint8_t msgType) {
    hdr->ver_type = ((version & 0x0F) << 4) | (msgType & 0x0F);
}

// Prototypy funkcji
int findClientByID(uint16_t cid);
void handleRegister(ALP_Header* hdr, const uint8_t* payload, const struct sockaddr_in* srcAddr, socklen_t srcLen);
void handleReport(ALP_Header* hdr, const uint8_t* payload, int payloadLen);
void bitsToString5(uint8_t, char *out);
uint32_t reverseBits(uint32_t value, int bitCount);

// funkcje statystyczne
uint32_t conwayLeading(uint8_t top, uint8_t bottom) 
{
    // wynik powstaje w 5 bitach 
    uint32_t result = 0;

    for (int step = 0; step < 5; step++) {
        // maska (5-step) bitów, np. step=0 => 5 bitów => mask=0x1F
        // step=1 => 4 bitów => mask=0x0F, ...
        uint8_t mask = (1 << (5 - step)) - 1;  // (1<<5=32) => 0x1F

        uint8_t bottomSub = bottom & mask; 
        uint8_t topSub    = top    & mask;

        int bitVal = 0;
        if (bottomSub == topSub) {
            bitVal = 1;
        }
        result |= (bitVal << step);

        bottom >>= 1;
    }
    return reverseBits(result, 5);
}

void printConwayFor2Clients(int c0, int c1)
{
    // wzorce
    uint8_t w1 = g_clients[c0].patternByte & 0x1F; // 5 bitów
    uint8_t w2 = g_clients[c1].patternByte & 0x1F;

    // Conway
    uint32_t c11 = conwayLeading(w1, w1);
    uint32_t c12 = conwayLeading(w1, w2);
    uint32_t c21 = conwayLeading(w2, w1);
    uint32_t c22 = conwayLeading(w2, w2);

    double numerator   = (double)((int)c22 - (int)c21);
    double denominator = (double)((int)c11 - (int)c12);
    double ratioP1P2   = 0.0;
    if (denominator == 0.0) {
        printf("UWAGA: c11-c12=0 -> ratioP1P2 nieliczalne.\n");
    } else {
        ratioP1P2 = numerator / denominator;
    }

    // P1+P2=1 => 
    // P1/P2= ratio => P1= ratio/(1+ratio), P2=1-P1
    double P1=0.0, P2=0.0;
    if (ratioP1P2 <= 0.0) {
        P1=0.0; 
        P2=1.0;
    } else {
        P1 = ratioP1P2 / (1.0 + ratioP1P2);
        P2 = 1.0 - P1;
    }

    // E = 2*(C11*C22 - C12*C21)/( (C11-C12) + (C22-C21) )
    double top    = 2.0 * ( (double)(c11*c22) - (double)(c12*c21) );
    double bottom = ( (double)((int)c11-(int)c12) + (double)((int)c22-(int)c21) );
    double E=0.0;
    if (bottom != 0.0) {
        E= top / bottom;
    }

    char w1str[6], w2str[6];
    bitsToString5(w1, w1str);
    bitsToString5(w2, w2str);

    printf("\n== Conway dla 2 klientow: c0=%d, c1=%d ==\n", c0, c1);
    printf("w1=%s, w2=%s\n", w1str, w2str);
    printf("  C11=%u, C12=%u, C21=%u, C22=%u\n", c11, c12, c21, c22);
    printf("  ratioP1P2=%.3f => P1=%.3f, P2=%.3f\n", ratioP1P2, P1, P2);
    printf("  E=%.3f\n", E);
    printf("========================================\n");
}

// Pomocnicze funkcje do wysy³ania i odbierania

// ta funkcja pomaga przekonwertowac odebrany pattern do postaci stringowej
void bitsToString5(uint8_t pattern, char* out)
{
    for (int i = 0; i < 5; i++) {
        int bitVal = (pattern >> (4 - i)) & 0x01;
        out[i] = bitVal ? '1' : '0';
    }
    out[5] = '\0';
}

// odbija lustrzanie liczbe binarna
uint32_t reverseBits(uint32_t value, int bitCount) {
    uint32_t reversed = 0;

    for (int i = 0; i < bitCount; i++) {
        uint32_t lsb = value & 1;

        reversed |= (lsb << (bitCount - 1 - i));

        value >>= 1;
    }

    return reversed;
}

int sendALPto(const void* data, size_t length, const struct sockaddr_in* addr, socklen_t addrLen) {
    ssize_t sent = sendto(g_serverSocket, data, length, 0, (struct sockaddr*)addr, addrLen);
    if (sent < 0) {
        perror("sendto");
        return -1;
    }
    g_totalSentPackets++;
    g_totalSentBytes += sent;
    return (int)sent;
}

void broadcastALP(const void* data, size_t length) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_clients[i].active) {
            sendALPto(data, length, &g_clients[i].addr, g_clients[i].addrLen);
        }
    }
}

void sendACK(uint16_t seqNum, uint16_t clientID, const struct sockaddr_in* addr, socklen_t addrLen) {
    uint8_t buffer[sizeof(ALP_Header)];
    memset(buffer, 0, sizeof(buffer));

    ALP_Header* hdr = (ALP_Header*)buffer;
    alp_setVersionType(hdr, 1, MSGT_ACK);
    hdr->flags = FLAG_ACK_RESPONSE; 
    hdr->seqNum = htons(seqNum);
    hdr->clientID = htons(clientID);
    hdr->payloadLength = htons(0);

    sendALPto(buffer, sizeof(ALP_Header), addr, addrLen);
}

void broadcastSimpleMessage(uint8_t msgType, const void* payload, uint16_t payloadLen) {
    uint8_t buffer[256];                    
    memset(buffer, 0, sizeof(buffer));

    ALP_Header* hdr = (ALP_Header*)buffer;      
    alp_setVersionType(hdr, 1, msgType);        
    hdr->flags = FLAG_ACK_REQUEST;            
    hdr->seqNum = htons(g_seqNum++);
    hdr->clientID = htons(0);                // 0 -> broadcast
    hdr->payloadLength = htons(payloadLen);

    if (payload && payloadLen > 0) {
        memcpy(buffer + sizeof(ALP_Header), payload, payloadLen);
    }

    size_t totalSize = sizeof(ALP_Header) + payloadLen;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_clients[i].active) {
            sendALPto(buffer, totalSize, &g_clients[i].addr, g_clients[i].addrLen);
        }
    }
}

// lobby
int waitForPlayers(int waitSec) {
    time_t start = time(NULL);
    time_t end   = start + waitSec;
    
    while (time(NULL) < end) {
        time_t remaining = end - time(NULL);

        struct timeval tv;
        tv.tv_sec = remaining;
        tv.tv_usec = 0;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(g_serverSocket, &readfds);

        int ret = select(g_serverSocket+1, &readfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(g_serverSocket, &readfds)) {
            uint8_t buffer[1500];
            struct sockaddr_in srcAddr;
            socklen_t srcLen = sizeof(srcAddr);
            ssize_t rec = recvfrom(g_serverSocket, buffer, sizeof(buffer), 0,
                                   (struct sockaddr*)&srcAddr, &srcLen);
            if (rec > 0) {
                g_totalRecvPackets++;
                g_totalRecvBytes += rec;

                if (rec >= (ssize_t)sizeof(ALP_Header)) {
                    ALP_Header* rhdr = (ALP_Header*)buffer;
                    uint16_t payLen = ntohs(rhdr->payloadLength);
                    uint8_t msgType = alp_getMsgType(rhdr);
                    const uint8_t* payPtr = buffer + sizeof(ALP_Header);

                    if (msgType == MSGT_ACK) {
                    }
                    else if (msgType == MSGT_REGISTER) {
                        handleRegister(rhdr, payPtr, &srcAddr, srcLen);
                    }
                    else if (msgType == MSGT_REPORT) {
                        handleReport(rhdr, payPtr, payLen);
                    }

                    if (rhdr->flags & FLAG_ACK_REQUEST) {
                        if (msgType != MSGT_REGISTER && msgType != MSGT_REPORT) {
                            sendACK(ntohs(rhdr->seqNum),
                                    ntohs(rhdr->clientID),
                                    &srcAddr, srcLen);
                        }
                    }

                    if(g_numClients < 2) {
                        return 0;
                    }

                }
            }
        }
    }
    if (g_numClients >= 2) {
        return 1;  
    } else {
        return 0;  
    }
}

int waitForAllACK(uint16_t expectedSeqNum, int waitSec) {
    int ackReceived[MAX_CLIENTS];
    memset(ackReceived, 0, sizeof(ackReceived));

    int totalActive = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_clients[i].active) {
            totalActive++;
        }
    }

    int ackCount = 0;
    time_t startTime = time(NULL);

    while((time(NULL) - startTime < waitSec)) {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(g_serverSocket, &readfds);

        int ret = select(g_serverSocket+1, &readfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(g_serverSocket, &readfds)) {
            uint8_t buffer[1500];
            struct sockaddr_in srcAddr;
            socklen_t srcLen = sizeof(srcAddr);
            ssize_t rec = recvfrom(g_serverSocket, buffer, sizeof(buffer), 0,
                                   (struct sockaddr*)&srcAddr, &srcLen);

            if (rec > 0) {
                g_totalRecvPackets++;
                g_totalRecvBytes += rec;

                if (rec >= (ssize_t)sizeof(ALP_Header)) {
                    ALP_Header* rhdr = (ALP_Header*)buffer;
                    uint8_t msgType = (rhdr->ver_type & 0x0F);
                    uint16_t seqNum = ntohs(rhdr->seqNum);
                    uint16_t cid    = ntohs(rhdr->clientID);

                    if ( (msgType == MSGT_ACK) && (seqNum == expectedSeqNum) ) {
                        int idx = findClientByID(cid);
                        if (idx >= 0 && g_clients[idx].active) {
                            if (!ackReceived[idx]) {
                                ackReceived[idx] = 1;
                                ackCount++;
                                printf("Odebrano ACK (seqNum=%u) od klienta ID=%d. [%d/%d]\n",
                                       seqNum, cid, ackCount, totalActive);
                                if (ackCount == totalActive) {
                                    return ackCount;
                                }
                            }
                        }
                    } else {
                        uint16_t payLen = ntohs(rhdr->payloadLength);
                        const uint8_t* payPtr = buffer + sizeof(ALP_Header);

                        if (rhdr->flags & FLAG_ACK_REQUEST) {
                            sendACK(ntohs(rhdr->seqNum), ntohs(rhdr->clientID),
                                    &srcAddr, srcLen);
                        }

                        if (msgType == MSGT_REGISTER) {
                            handleRegister(rhdr, payPtr, &srcAddr, srcLen);
                        }
                        else if (msgType == MSGT_REPORT) {
                            handleReport(rhdr, payPtr, payLen);
                        }
                    }
                }
            }
        }
    }

    return ackCount;
}

// Funkcje pomocnicze do znajdowania/dodawania klienta
int findClientByID(uint16_t cid) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_clients[i].active && g_clients[i].clientID == cid) {
            return i;
        }
    }
    return -1;
}

int addNewClient(const struct sockaddr_in* addr, socklen_t addrLen) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!g_clients[i].active) {
            g_clients[i].active = 1;
            g_clients[i].clientID = g_nextClientID++;
            g_clients[i].addr = *addr;
            g_clients[i].addrLen = addrLen;
            g_clients[i].wins = 0;
            g_clients[i].patternByte = 0;
            g_clients[i].patternLen = 0;
            g_clients[i].totalTossesForWins = 0;
            g_numClients++;
            return i;
        }
    }
    return -1;
}

// Rejestracja
void handleRegister(ALP_Header* hdr, const uint8_t* payload, const struct sockaddr_in* srcAddr, socklen_t srcLen) {
    int idx = addNewClient(srcAddr, srcLen);
    if (idx < 0) {
        fprintf(stderr, "Brak miejsca na nowego klienta.\n");
        return;
    }
    uint16_t newCID = g_clients[idx].clientID;
    printf("Nowy klient zarejestrowany: ID=%u\n", newCID);
    uint16_t payLen = ntohs(hdr->payloadLength);
    if(payLen >= 1) {
        g_clients[idx].patternByte = payload[0]; 
        g_clients[idx].patternLen = 5; 
    }
    sendACK(ntohs(hdr->seqNum), newCID, srcAddr, srcLen);
}

// Report
static int g_currentWinnerClientID = -1;
static int g_currentWinningTossCount = 999999999;

void handleReport(ALP_Header* hdr, const uint8_t* payload, int payloadLen) {
    if (payloadLen < 8) return;
    uint32_t game_id;
    uint32_t toss_count;
    memcpy(&game_id, payload, 4);
    memcpy(&toss_count, payload+4, 4);
    game_id = ntohl(game_id);
    toss_count = ntohl(toss_count);

    int cindex = findClientByID(ntohs(hdr->clientID));
    if (cindex < 0) return;

    if ((int)toss_count < g_currentWinningTossCount) {
        g_currentWinnerClientID = g_clients[cindex].clientID;
        g_currentWinningTossCount = toss_count;
    }

    sendACK(ntohs(hdr->seqNum), ntohs(hdr->clientID), &g_clients[cindex].addr, g_clients[cindex].addrLen);
}


// pojedyncza gra
void make_one_game(int gameID) {
    printf("\n--- Rozpoczynam gre #%d ---\n", gameID);

    g_currentWinnerClientID = -1;
    g_currentWinningTossCount = 999999999;

    uint16_t startSeqNum = g_seqNum;

    uint8_t payload[4];
    uint32_t gid = htonl(gameID);
    memcpy(payload, &gid, 4);
    broadcastSimpleMessage(MSGT_START, payload, 4);
    

    int clients_start_acked = waitForAllACK(startSeqNum, START_ACK_TIMEOUT);
    printf("Otrzymane potwierdzenia START: %d/%d\n", clients_start_acked, g_numClients);

    int tossCount = 0;

    while (1) {         
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(g_serverSocket, &readfds);

        int ret = select(g_serverSocket+1, &readfds, NULL, NULL, &tv);
        if (ret > 0 && FD_ISSET(g_serverSocket, &readfds)) {
            uint8_t buffer[1500];
            struct sockaddr_in srcAddr;
            socklen_t srcLen = sizeof(srcAddr);
            ssize_t rec = recvfrom(g_serverSocket, buffer, sizeof(buffer), 0,
                                   (struct sockaddr*)&srcAddr, &srcLen);
            if (rec > 0) {
                g_totalRecvPackets++;
                g_totalRecvBytes += rec;

                if (rec >= (ssize_t)sizeof(ALP_Header)) {
                    ALP_Header* rhdr = (ALP_Header*)buffer;
                    uint16_t payLen = ntohs(rhdr->payloadLength);
                    uint8_t msgType = alp_getMsgType(rhdr);
                    const uint8_t* payPtr = buffer + sizeof(ALP_Header);

                    if (msgType == MSGT_ACK) {
                    } 
                    else if (msgType == MSGT_REGISTER) {
                        handleRegister(rhdr, payPtr, &srcAddr, srcLen);
                    }
                    else if (msgType == MSGT_REPORT) {
                        handleReport(rhdr, payPtr, payLen);
                    }

                    if (rhdr->flags & FLAG_ACK_REQUEST) {
                        sendACK(ntohs(rhdr->seqNum), ntohs(rhdr->clientID), &srcAddr, srcLen);
                    }
                }
            }
        }

        if (g_currentWinnerClientID >= 0) {
            
            struct timeval extraWait;
            extraWait.tv_sec = 0;
            extraWait.tv_usec = 200000; // 200ms

            fd_set readfds2;
            FD_ZERO(&readfds2);
            FD_SET(g_serverSocket, &readfds2);

            int ret2 = select(g_serverSocket+1, &readfds2, NULL, NULL, &extraWait);
            if (ret2 > 0 && FD_ISSET(g_serverSocket, &readfds2)) {
                uint8_t buffer[1500];
                struct sockaddr_in srcAddr;
                socklen_t srcLen = sizeof(srcAddr);
                ssize_t rec = recvfrom(g_serverSocket, buffer, sizeof(buffer), 0,
                                    (struct sockaddr*)&srcAddr, &srcLen);
                if (rec > 0) {
                    g_totalRecvPackets++;
                    g_totalRecvBytes += rec;

                    if (rec >= (ssize_t)sizeof(ALP_Header)) {
                        ALP_Header* rhdr = (ALP_Header*)buffer;
                        uint16_t payLen = ntohs(rhdr->payloadLength);
                        uint8_t msgType = alp_getMsgType(rhdr);
                        const uint8_t* payPtr = buffer + sizeof(ALP_Header);

                        if (msgType == MSGT_REPORT) {
                            handleReport(rhdr, payPtr, payLen);
                        }
                        if (rhdr->flags & FLAG_ACK_REQUEST) {
                            sendACK(ntohs(rhdr->seqNum), 
                                    ntohs(rhdr->clientID),
                                    &srcAddr, srcLen);
                        }
                    }
                }
            }

            break;
        }


        tossCount++;
        uint8_t coin = rand() % 2;

        uint16_t tossSeqNum = g_seqNum;
        broadcastSimpleMessage(MSGT_TOSS, &coin, 1);

        int clients_toss_acked = waitForAllACK(tossSeqNum, TOSS_ACK_TIMEOUT);
        printf("Otrzymane potwierdzenia TOSS: %d/%d\n", clients_start_acked, g_numClients);
    }

    char winner_pattern[6];
    int winner_idx = findClientByID(g_currentWinnerClientID);
    bitsToString5(g_clients[winner_idx].patternByte, winner_pattern);
    printf("Zwyciezca: clientID=%d, tossCount=%d, pattern=%s\n", g_currentWinnerClientID, g_currentWinningTossCount, winner_pattern);

    g_gameRecords[g_currentGameIndex].gameID = gameID;
    g_gameRecords[g_currentGameIndex].winnerClientID = g_currentWinnerClientID;
    g_gameRecords[g_currentGameIndex].winningTossCount = g_currentWinningTossCount;
    g_currentGameIndex++;

    int idx = findClientByID(g_currentWinnerClientID);
    if (idx >= 0) {
        g_clients[idx].wins++;
        g_clients[idx].totalTossesForWins += tossCount;
    }

    {
        uint8_t payload[8];
        uint32_t gid = htonl(gameID);
        uint32_t wid = htonl(g_currentWinnerClientID);
        memcpy(payload, &gid, 4);
        memcpy(payload+4, &wid, 4);
        broadcastSimpleMessage(MSGT_ENDGAME, payload, 8);
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Uzycie: %s <port> <num_games>\n", argv[0]);
        return 1;
    }

    int port = atoi(argv[1]);
    int num_games = atoi(argv[2]);
    if (num_games > MAX_GAMES) num_games = MAX_GAMES;

    srand(time(NULL));

    g_serverSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_serverSocket < 0) {
        perror("socket");
        return 1;
    }

    memset(&g_serverAddr, 0, sizeof(g_serverAddr));
    g_serverAddr.sin_family = AF_INET;
    g_serverAddr.sin_addr.s_addr = INADDR_ANY;
    g_serverAddr.sin_port = htons(port);

    if (bind(g_serverSocket, (struct sockaddr*)&g_serverAddr, sizeof(g_serverAddr)) < 0) {
        perror("bind");
        close(g_serverSocket);
        return 1;
    }

    printf("Serwer uruchomiony na porcie %d.\n", port);

    memset(g_clients, 0, sizeof(g_clients));
    memset(g_gameRecords, 0, sizeof(g_gameRecords));

    while(1) {

        printf("Oczekiwanie na min. 2 graczy przez 20s...\n");
        int at_least_2_players_in_lobby = waitForPlayers(20);

        if(!at_least_2_players_in_lobby && g_numClients > 0) {
            printf("Za malo graczy, wyrejestrowywanie klienta oraz powrot do lobby...\n");
            memset(g_clients, 0, sizeof(g_clients));
            g_numClients = 0;
            continue;
        } else if(!at_least_2_players_in_lobby || g_numClients < 2) { 
            printf("Za malo graczy, powrot do lobby...\n");
            memset(g_clients, 0, sizeof(g_clients));
            g_numClients = 0;
            continue;

        }

        // g³ówna pêtla gier
        for (int i = 1; i <= num_games; i++) {
            make_one_game(i);
            printf("Zakonczono gre #%d.\n", i);
        }
        break;
    }

    // statystyki
    printf("\n=== Statystyki ===\n");
    printf("Liczba klientow: %d\n", g_numClients);
    printf("Dlugosc sekwencji binarnej uczestnikow: %d\n", g_clients[g_numClients-1].patternLen); // dlugosc patternu ostatniego clienta, ale kazdy ma taka sama
    printf("Rozegrano gier: %d\n", num_games);

    long sumTosses = 0;
    for (int i = 0; i < num_games; i++) {
        sumTosses += g_gameRecords[i].winningTossCount;
    }
    double avgToss = (double)sumTosses / num_games;
    printf("Srednia liczba rzutow do wygranej: %.2f\n", avgToss);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (g_clients[i].active) {
            double avg = 0.0;
            if (g_clients[i].wins > 0) {
                avg = (double)g_clients[i].totalTossesForWins / g_clients[i].wins;
            }
            printf("ClientID=%u wygrane=%d, srednia liczba rzutow do wygranej: %.2f\n",
                   g_clients[i].clientID, g_clients[i].wins, avg);
        }
    }

    // Conway
    if (g_numClients == 2) {
        printConwayFor2Clients(0, 1);
    } else {
        printf("Conway Leading: mam %d klientow, a obsluga wymaga 2.\n", g_numClients);
    }

    // Metryki transmisji
    printf("\nTransmisje:\n");
    printf("  Wyslane pakiety:  %ld\n", g_totalSentPackets);
    printf("  Odebrane pakiety: %ld\n", g_totalRecvPackets);
    if (g_totalSentPackets > 0)
        printf("  Sredni rozmiar wysylanego pakietu: %.1f bajtow\n", (double)g_totalSentBytes / g_totalSentPackets);
    if (g_totalRecvPackets > 0)
        printf("  Sredni rozmiar odebranego pakietu: %.1f bajtow\n", (double)g_totalRecvBytes / g_totalRecvPackets);

    close(g_serverSocket);
    return 0;
}