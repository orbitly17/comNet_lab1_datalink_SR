#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 1200
#define ACK_TIMER 110
#define MAX_SEQ 31
#define NR_BUFS ((MAX_SEQ + 1) / 2)
#define inc(k) if(k < MAX_SEQ) k++; else k=0

struct FRAME {
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};

struct packet {
    unsigned char data[PKT_LEN];
};

static unsigned char frame_nr = 0, nbuffered;
static unsigned char frame_expected = 0;
static int phl_ready = 0;
static int no_nak = 1;

static int between(unsigned char a, unsigned char b, unsigned char c) {
    return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

static void put_frame(unsigned char* frame, int len) {
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;
}

static void send_the_frame(unsigned char kind, unsigned char frame_nr, unsigned char frame_expceted, struct packet buffer[])
{
    struct FRAME s;
    s.kind = kind;
    if (kind == FRAME_DATA) {
        memcpy(s.data, buffer[frame_nr % NR_BUFS].data, PKT_LEN);
        s.seq = frame_nr;
        dbg_frame("Send DATA %d %d, ID %d\n", frame_nr, ((frame_expceted + MAX_SEQ) % (MAX_SEQ + 1)), *(short*)s.data);
    }
    s.ack = ((frame_expceted + MAX_SEQ) % (MAX_SEQ + 1));
    if (kind == FRAME_NAK) {
        dbg_frame("Send NAK %d\n", frame_expceted);
        no_nak = 0;
    }
    if (kind == FRAME_ACK) {
        dbg_frame("Send ACK %d\n", frame_expceted);
    }
    if (kind == FRAME_ACK || kind == FRAME_NAK) {
        send_frame((unsigned char*)&s, 2);
    }
    else {
        put_frame((unsigned char*)&s, 3 + PKT_LEN);
    }
    if (kind == FRAME_DATA) {
        start_timer(frame_nr % NR_BUFS, DATA_TIMER);
    }
    stop_ack_timer();
}

int main(int argc, char** argv)
{
    int event, oldest_frame;
    struct FRAME f;
    int len = 0;
    struct packet out_buffer[NR_BUFS];
    struct packet in_buffer[NR_BUFS];
    unsigned char ack_expected = 0;
    unsigned char next_frame_to_send = 0;
    unsigned char frame_expected = 0;
    unsigned char too_far = NR_BUFS;
    int arrived[NR_BUFS] = { 0 };

    protocol_init(argc, argv);
    lprintf("Designed by Bonbon, build: " __DATE__ "  "__TIME__"\n");

    disable_network_layer();

    while (1) {
        event = wait_for_event(&oldest_frame);

        switch (event) {
            case NETWORK_LAYER_READY:
                get_packet(out_buffer[next_frame_to_send % NR_BUFS].data);
                nbuffered++;
                send_the_frame(FRAME_DATA, next_frame_to_send, frame_expected, out_buffer);
                inc(next_frame_to_send);
                break;

            case PHYSICAL_LAYER_READY:
                phl_ready = 1;
                break;

            case FRAME_RECEIVED:
                len = recv_frame((unsigned char*)&f, sizeof f);

                if ((len < 7 && len != 2) || (len >= 7 && crc32((unsigned char*)&f, len) != 0)) {
                    dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                    if (no_nak == 1) {
                        send_the_frame(FRAME_NAK, 0, frame_expected, out_buffer);
                    }
                    break;
                }

                if (f.kind == FRAME_ACK)
                    dbg_frame("Recv ACK  %d\n", f.ack);

                if (f.kind == FRAME_DATA) {
                    dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
                    if (f.seq != frame_expected && no_nak == 1) {
                        send_the_frame(FRAME_NAK, 0, frame_expected, out_buffer);
                    }
                    else {
                        start_ack_timer(ACK_TIMER);
                    }
                    if (between(frame_expected, f.seq, too_far) && arrived[f.seq % NR_BUFS] == 0) {
                        arrived[f.seq % NR_BUFS] = 1;
                        memcpy(in_buffer[f.seq % NR_BUFS].data, f.data, PKT_LEN);
                        while (arrived[frame_expected % NR_BUFS]) {
                            put_packet(in_buffer[frame_expected % NR_BUFS].data, PKT_LEN);
                            arrived[frame_expected % NR_BUFS] = 0;
                            inc(frame_expected);
                            inc(too_far);
                            no_nak = 1;
                            start_ack_timer(ACK_TIMER);
                        }
                    }
                }

                if (f.kind == FRAME_NAK) {
                    dbg_frame("Recv NAK  %d\n", f.ack);
                    if (between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) {
                        send_the_frame(FRAME_DATA, (f.ack + 1) % (MAX_SEQ + 1), frame_expected, out_buffer);
                    }
                }

                while (between(ack_expected, f.ack, next_frame_to_send)) {
                    stop_timer(ack_expected % NR_BUFS);
                    inc(ack_expected);
                    nbuffered--;
                }

                break;

            case DATA_TIMEOUT:
                dbg_event("---- DATA %d timeout\n", oldest_frame);
                if (!between(ack_expected, oldest_frame, next_frame_to_send)) {
                    oldest_frame += NR_BUFS;
                }
                send_the_frame(FRAME_DATA, oldest_frame, frame_expected, out_buffer);
                break;

            case ACK_TIMEOUT:
                dbg_event("---- ACK %d timeout\n", oldest_frame);
                send_the_frame(FRAME_ACK, 0, frame_expected, out_buffer);
                break;
        }


        if (nbuffered < NR_BUFS && phl_ready == 1)
            enable_network_layer();
        else
            disable_network_layer();
    }
}