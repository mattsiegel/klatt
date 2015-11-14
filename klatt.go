/*
An implementation of a Klatt cascade-parallel formant synthesizer.

Version: 1.0
Date: 2014-02-24

This is a grandchild of Dennis Klatt's implementation.
Its direct parent is Jon Iles's and Nick Ing-Simmons's C version from 1994.
I've reinstated some original comments that had been lost over time.

See the README file for further details.

(c) 2014 Matthew Siegel

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 1, or (at your option)
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

//package klatt
package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"github.com/vova616/go-openal/openal"
	"io"
	"log"
	"math"
	"math/rand"
	"os"
	//	"runtime/pprof"
	"strings"
	"time"
)

func main() {
	var err error

	/*
		pf, err := os.Create("go_klatt.prof")
		if err != nil {
			log.Fatal(err)
		}
		pprof.StartCPUProfile(pf)
		defer pprof.StopCPUProfile()
	*/
	in, err := os.Open("example2.par")
	if err != nil {
		log.Fatal(err)
	}
	defer in.Close()

	out, err := os.Create("go_klatt_output.raw")
	if err != nil {
		log.Fatal(err)
	}
	defer out.Close()

	device := openal.OpenDevice("")
	context := device.CreateContext()
	context.Activate()
	source := openal.NewSource()
	source.SetPitch(0.08)
	source.SetGain(1)
	source.SetPosition(0, 0, 0)
	source.SetVelocity(0, 0, 0)
	source.SetLooping(false)
	albuffer := openal.NewBuffer()

	start := time.Now()
	synth := NewSynthesizer()
	var tempbuf bytes.Buffer
	for {
		frame, err := readFrame(in)
		if err == io.EOF {
			break
		}
		if err != nil {
			log.Fatal(err)
		}

		err = synth.Render(frame, &tempbuf)
		if err != nil {
			log.Fatal(err)
		}

		/*
			_, err = tempbuf.WriteTo(out)
			if err != nil {
				log.Fatal(err)
			}
		*/
	}
	fmt.Println("render elapsed:", time.Since(start))

	albuffer.SetData(openal.FormatMono16, tempbuf.Bytes(), int32(tempbuf.Len()))
	source.SetBuffer(albuffer)
	start = time.Now()
	source.Play()
	for source.State() == openal.Playing {
		//loop long enough to let the wave file finish
	}
	fmt.Println("play elapsed:", time.Since(start))
	source.Pause()
	source.Stop()
	context.Destroy()
}

func readFrame(reader io.Reader) (frame Frame, err error) {
	_, err = fmt.Fscanf(reader, strings.Repeat("%v ", 40),
		&frame.F0hz10,
		&frame.AVdb,
		&frame.F1hz,
		&frame.B1hz,
		&frame.F2hz,
		&frame.B2hz,
		&frame.F3hz,
		&frame.B3hz,
		&frame.F4hz,
		&frame.B4hz,
		&frame.F5hz,
		&frame.B5hz,
		&frame.F6hz,
		&frame.B6hz,
		&frame.FNZhz,
		&frame.BNZhz,
		&frame.FNPhz,
		&frame.BNPhz,
		&frame.ASP,
		&frame.Kopen,
		&frame.Aturb,
		&frame.TLTdb,
		&frame.AF,
		&frame.Kskew,
		&frame.A1,
		&frame.B1phz,
		&frame.A2,
		&frame.B2phz,
		&frame.A3,
		&frame.B3phz,
		&frame.A4,
		&frame.B4phz,
		&frame.A5,
		&frame.B5phz,
		&frame.A6,
		&frame.B6phz,
		&frame.ANP,
		&frame.AB,
		&frame.AVpdb,
		&frame.Gain0,
	)
	return frame, err
}

// Synthesis models
const (
	CascadeParallel = iota
	AllParallel
)

// Voicing sources
const (
	Natural = iota
	Impulsive
	Sampled
)

type Resonator struct {
	a  float64
	b  float64
	c  float64
	p1 float64
	p2 float64
}

type Synthesizer struct {
	ByteOrder      binary.ByteOrder // we write int16s to the output; ordinarily they come out least significant byte first
	QuietErrors    bool             // suppress error logging
	SynthesisModel int              // cascade-parallel (default) or all-parallel

	outsl int // Output waveform selector

	// Counters

	nper int // Current location within the voicing period   40000 samp/s
	ns   int // Current sample within frame (Generator loop counter)

	// Counter limits

	t0    int // (T0) Fundamental period in output samples times 4
	nopen int // Number of samples in open phase of period
	nmod  int // Position in period to begin noise amp. modul

	// Various amplitude variables used in main loop

	ampVoice    float64 // AVdb converted to linear gain
	ampBypass   float64 // AB converted to linear gain
	parAmpVoice float64 // AVpdb converted to linear gain
	ampAspir    float64 // AP converted to linear gain
	ampFric     float64 // AF converted to linear gain
	ampBreath   float64 // ATURB converted to linear gain

	// State variables of sound sources

	onemd    float64 // in voicing one-pole low-pass filter
	decay    float64 // TLTdb converted to exponential time const
	minusPiT float64 // func. of sample rate
	twoPiT   float64 // func. of sample rate

	// ...

	ampGain0       float64 // G0 converted to linear gain
	sampleFactor   float64 // multiplication factor for glottal samples
	naturalSamples []int   // glottal samples

	sampleRate      int     // Number of output samples per second
	samplesPerFrame int     // number of samples per frame
	glotLPFreq      float64 // Frequency of glottal downsample low-pass filter, in Hz
	glotLPBandw     float64 // Bandwidth of glottal downsample low-pass filter, in Hz
	glsource        int     // Type of glottal source
	nfcascade       int     // Number of formants in cascade vocal tract
	f0Flutter       int     // Percentage of f0 flutter 0-100
	originalF0      int     // original value of f0 not modified by flutter
	nrand           int     // Varible used by random number generator
	natglotA        float64 // (natglot_a) Makes waveshape of glottal pulse when open
	natglotB        float64 // (natglot_b) Makes waveshape of glottal pulse when open

	rnpp *Resonator // parallel nasal pole
	r1p  *Resonator // parallel 1st formant
	r2p  *Resonator // parallel 2nd formant
	r3p  *Resonator // parallel 3rd formant
	r4p  *Resonator // parallel 4th formant
	r5p  *Resonator // parallel 5th formant
	r6p  *Resonator // parallel 6th formant
	r1c  *Resonator // cascade 1st formant
	r2c  *Resonator // cascade 2nd formant
	r3c  *Resonator // cascade 3rd formant
	r4c  *Resonator // cascade 4th formant
	r5c  *Resonator // cascade 5th formant
	r6c  *Resonator // cascade 6th formant
	r7c  *Resonator // cascade 7th formant
	r8c  *Resonator // cascade 8th formant
	rnpc *Resonator // cascade nasal pole
	rnz  *Resonator // cascade nasal zero
	rgl  *Resonator // crit-damped glot low-pass filter
	rlp  *Resonator // downsamp low-pass filter
	rout *Resonator // output low-pass

	// in the C version these were static variables
	// they're consolidated from several functions

	flutterTimeCount int
	lastNoise        float64
	vwave            float64
	skew             int

	// private state variables that persist between calls to Render
	// (in the C version these were static variables inside Render (aka "parwave()")

	renderStatics struct {
		noise    float64
		voice    float64
		source   float64
		vlast    float64
		glotlast float64
	}
}

// Synthesizer parameters for generating a frame of audio
type Frame struct {
	F0hz10 int     // Voicing fund freq in Hz
	AVdb   int     // Amp of voicing in dB,            0 to   70
	F1hz   float64 // First formant freq in Hz,        200 to 1300
	B1hz   float64 // First formant bw in Hz,          40 to 1000
	F2hz   float64 // Second formant freq in Hz,       550 to 3000
	B2hz   float64 // Second formant bw in Hz,         40 to 1000
	F3hz   float64 // Third formant freq in Hz,        1200 to 4999
	B3hz   float64 // Third formant bw in Hz,          40 to 1000
	F4hz   float64 // Fourth formant freq in Hz,       1200 to 4999
	B4hz   float64 // Fourth formant bw in Hz,         40 to 1000
	F5hz   float64 // Fifth formant freq in Hz,        1200 to 4999
	B5hz   float64 // Fifth formant bw in Hz,          40 to 1000
	F6hz   float64 // Sixth formant freq in Hz,        1200 to 4999
	B6hz   float64 // Sixth formant bw in Hz,          40 to 2000
	FNZhz  float64 // Nasal zero freq in Hz,           248 to  528
	BNZhz  float64 // Nasal zero bw in Hz,             40 to 1000
	FNPhz  float64 // Nasal pole freq in Hz,           248 to  528
	BNPhz  float64 // Nasal pole bw in Hz,             40 to 1000
	ASP    int     // Amp of aspiration in dB,         0 to   70
	Kopen  int     // # of samples in open period,     10 to   65
	Aturb  int     // Breathiness in voicing,          0 to   80
	TLTdb  int     // Voicing spectral tilt in dB,     0 to   24
	AF     int     // Amp of frication in dB,          0 to   80
	Kskew  int     // Skewness of alternate periods,   0 to   40 in sample#/2
	A1     int     // Amp of par 1st formant in dB,    0 to   80
	B1phz  float64 // Par. 1st formant bw in Hz,       40 to 1000
	A2     int     // Amp of F2 frication in dB,       0 to   80
	B2phz  float64 // Par. 2nd formant bw in Hz,       40 to 1000
	A3     int     // Amp of F3 frication in dB,       0 to   80
	B3phz  float64 // Par. 3rd formant bw in Hz,       40 to 1000
	A4     int     // Amp of F4 frication in dB,       0 to   80
	B4phz  float64 // Par. 4th formant bw in Hz,       40 to 1000
	A5     int     // Amp of F5 frication in dB,       0 to   80
	B5phz  float64 // Par. 5th formant bw in Hz,       40 to 1000
	A6     int     // Amp of F6 (same as r6pa),        0 to   80
	B6phz  float64 // Par. 6th formant bw in Hz,       40 to 2000
	ANP    int     // Amp of par nasal pole in dB,     0 to   80
	AB     int     // Amp of bypass fric. in dB,       0 to   80
	AVpdb  int     // Amp of voicing,  par in dB,      0 to   70
	Gain0  int     // Overall gain, 60 dB is unity,    0 to   60
}

// Generic resonator step function.
func (r *Resonator) resonator(input float64) float64 {
	x := r.a*input + r.b*r.p1 + r.c*r.p2
	r.p2 = r.p1
	r.p1 = x
	return x
}

/*
	Generic anti-resonator step function. The code is the same as resonator()
	except that a,b,c need to be set with setZeroABC() and we save inputs in
	p1/p2 rather than outputs. There is currently only one of these - "rnz"
	Output = (rnz.a * input) + (rnz.b * oldin1) + (rnz.c * oldin2)
*/
func (r *Resonator) antiresonator(input float64) float64 {
	x := r.a*input + r.b*r.p1 + r.c*r.p2
	r.p2 = r.p1
	r.p1 = input
	return x
}

func NewSynthesizer() *Synthesizer {
	sampleRate := 16000
	minusPiT := -math.Pi / float64(sampleRate)

	s := &Synthesizer{
		ByteOrder:       binary.LittleEndian,
		sampleRate:      sampleRate,
		samplesPerFrame: sampleRate / 200,
		glotLPFreq:      float64((950 * sampleRate) / 10000),
		glotLPBandw:     float64((630 * sampleRate) / 10000),
		minusPiT:        minusPiT,
		twoPiT:          -2 * minusPiT,
		nfcascade:       8,
		f0Flutter:       10,
		rnpp:            &Resonator{},
		r1p:             &Resonator{},
		r2p:             &Resonator{},
		r3p:             &Resonator{},
		r4p:             &Resonator{},
		r5p:             &Resonator{},
		r6p:             &Resonator{},
		r1c:             &Resonator{},
		r2c:             &Resonator{},
		r3c:             &Resonator{},
		r4c:             &Resonator{},
		r5c:             &Resonator{},
		r6c:             &Resonator{},
		r7c:             &Resonator{},
		r8c:             &Resonator{},
		rnpc:            &Resonator{},
		rnz:             &Resonator{},
		rgl:             &Resonator{},
		rlp:             &Resonator{},
		rout:            &Resonator{},
	}
	s.setABC(s.rlp, s.glotLPFreq, s.glotLPBandw)
	return s
}

// Convert synthesis parameters to a waveform and output it.
func (s *Synthesizer) Render(frameParams Frame, writer io.Writer) (err error) {
	t := &s.renderStatics // in the C version, these were static vars of this function ("parwave")

	frame := &frameParams // we'll be modifying frameParams that was passed to us

	s.frameInit(frame) // get parameters for next frame of speech

	if s.f0Flutter != 0 {
		s.flutter(frame) // add f0 flutter
	}

	// MAIN LOOP, for each output sample of current frame:
	for s.ns = 0; s.ns < s.samplesPerFrame; s.ns++ {

		/*
			Get low-passed random number for aspiration and frication noise

			Noise spectrum is tilted down by soft low-pass filter having a pole near
			the origin in the z-plane, i.e. output = input + (0.75 * lastoutput)
		*/
		s.nrand = rand.Intn(16382) - 8191 // -8191 to +8191
		t.noise = float64(s.nrand) + 0.75*s.lastNoise
		s.lastNoise = t.noise

		/*
		   Amplitude modulate noise (reduce noise amplitude during
		   second half of glottal period) if voicing simultaneously present.
		*/
		if s.nper > s.nmod {
			t.noise *= 0.5
		}

		// Compute frication noise
		frics := s.ampFric * t.noise

		/*
		   Compute voicing waveform: (run glottal source simulation at 4
		   times normal sample rate to minimize quantization noise in
		   period of female voice)
		*/

		for n4 := 0; n4 < 4; n4++ {

			// Choose glottal source
			switch s.glsource {
			case Impulsive:
				// Use impulsive glottal source
				t.voice = s.impulsiveSource()
			case Natural:
				// Or use a more-natural-shaped source waveform with excitation
				// occurring both upon opening and upon closure, strongest at closure
				t.voice = s.naturalSource()
			case Sampled:
				t.voice = s.sampledSource()
			}

			// Reset period when counter 'loc' reaches t0
			if s.nper >= s.t0 {
				s.nper = 0
				s.resetParams(frame)
			}

			/*
				Low-pass filter voicing waveform before downsampling from 4*sampleRate
				to sampleRate samples/sec.  Resonator f=.09*sampleRate, bw=.06*sampleRate
			*/
			t.voice = s.rlp.resonator(t.voice) // in=voice, out=voice

			// Increment counter that keeps track of 4*sampleRate samples per sec

			s.nper++
		}

		/*
		   Tilt spectrum of voicing source down by soft low-pass filtering,
		   amount of tilt determined by TLTdb
		*/
		t.voice = (t.voice * s.onemd) + (t.vlast * s.decay)
		t.vlast = t.voice

		/*
		   Add breathiness during glottal open phase. Amount of breathiness
		   determined by parameter Aturb Use nrand rather than noise because
		   noise is low-passed.
		*/
		if s.nper < s.nopen {
			t.voice += s.ampBreath * float64(s.nrand)
		}

		// Set amplitude of voicing

		glotout := s.ampVoice * t.voice
		par_glotout := s.parAmpVoice * t.voice

		// Compute aspiration amplitude and add to voicing source

		aspiration := s.ampAspir * t.noise
		glotout += aspiration
		par_glotout += aspiration

		/*
		   Cascade vocal tract, excited by laryngeal sources.
		   Nasal antiresonator, then formants FNP, F5, F4, F3, F2, F1
		*/

		var out float64
		if s.SynthesisModel == CascadeParallel {
			rnzout := s.rnz.antiresonator(glotout)   // Output of cascade nazal zero resonator
			casc_next_in := s.rnpc.resonator(rnzout) // in=rnzout, out=rnpc.p1

			// Keep all formants enabled; assumes sample rate >= 16000

			if s.nfcascade >= 8 {
				casc_next_in = s.r8c.resonator(casc_next_in)
			}
			if s.nfcascade >= 7 {
				casc_next_in = s.r7c.resonator(casc_next_in)
			}
			if s.nfcascade >= 6 {
				casc_next_in = s.r6c.resonator(casc_next_in)
			}
			if s.nfcascade >= 5 {
				casc_next_in = s.r5c.resonator(casc_next_in)
			}
			if s.nfcascade >= 4 {
				casc_next_in = s.r4c.resonator(casc_next_in)
			}
			if s.nfcascade >= 3 {
				casc_next_in = s.r3c.resonator(casc_next_in)
			}
			if s.nfcascade >= 2 {
				casc_next_in = s.r2c.resonator(casc_next_in)
			}
			if s.nfcascade >= 1 {
				out = s.r1c.resonator(casc_next_in)
			}
		}

		// Excite parallel F1 and FNP by voicing waveform

		t.source = par_glotout // Source is voicing plus aspiration

		out += s.r1p.resonator(t.source)
		out += s.rnpp.resonator(t.source)

		/*
			Sound source for other parallel resonators is frication plus
			first difference of voicing waveform.
		*/
		t.source = frics + par_glotout - t.glotlast
		t.glotlast = par_glotout

		/*
			Standard parallel vocal tract.
			Formants F6,F5,F4,F3,F2, outputs added with alternating sign.
		*/
		out = s.r6p.resonator(t.source) - out
		out = s.r5p.resonator(t.source) - out
		out = s.r4p.resonator(t.source) - out
		out = s.r3p.resonator(t.source) - out
		out = s.r2p.resonator(t.source) - out

		outbypas := s.ampBypass * t.source
		out = outbypas - out

		//!! outsl is not set anywhere
		if s.outsl != 0 {
			switch s.outsl {
			case 1:
				out = t.voice
			case 2:
				out = aspiration
			case 3:
				out = frics
			case 4:
				out = glotout
			case 5:
				out = par_glotout
			case 6:
				out = outbypas
			case 7:
				out = t.source
			}
		}

		out = s.rout.resonator(out)

		// clip, convert to integer, output
		b := clip(out * s.ampGain0)
		err := binary.Write(writer, s.ByteOrder, b)
		if err != nil {
			return err
		}
	}

	return nil
}

// Clip on boundaries of 16-bit word and return as integer.
func clip(s float64) int16 {
	switch {
	case s < -32767:
		return -32767
	case s > 32767:
		return 32767
	}
	return int16(s)
}

// Use parameters from the input frame to set up resonator coefficients.
func (s *Synthesizer) frameInit(frame *Frame) {
	/*
		Read speech frame definition into temp store
		and move some parameters into active use immediately
		(voice-excited ones are updated pitch synchronously
		to avoid waveform glitches).
	*/

	s.originalF0 = frame.F0hz10 / 10

	frame.AVdb -= 7
	if frame.AVdb < 0 {
		frame.AVdb = 0
	}

	s.ampAspir = DBtoLIN(frame.ASP) * 0.05
	s.ampFric = DBtoLIN(frame.AF) * 0.25

	s.parAmpVoice = DBtoLIN(frame.AVpdb)

	/*
		Fudge factors (which comprehend affects of formants on each other?)
		With these in place ALL_PARALLEL should sound as close as  possible to CASCADE_PARALLEL.
		Possible problem feeding in Holmes's amplitudes given this.
	*/
	amp_parF1 := DBtoLIN(frame.A1) * 0.4   // -7.96 dB
	amp_parF2 := DBtoLIN(frame.A2) * 0.15  // -16.5 dB
	amp_parF3 := DBtoLIN(frame.A3) * 0.06  // -24.4 dB
	amp_parF4 := DBtoLIN(frame.A4) * 0.04  // -28.0 dB
	amp_parF5 := DBtoLIN(frame.A5) * 0.022 // -33.2 dB
	amp_parF6 := DBtoLIN(frame.A6) * 0.03  // -30.5 dB
	amp_parFNP := DBtoLIN(frame.ANP) * 0.6 // -4.44 dB
	s.ampBypass = DBtoLIN(frame.AB) * 0.05 // -26.0 db

	frame.Gain0 -= 3
	if frame.Gain0 <= 0 {
		frame.Gain0 = 57
	}
	s.ampGain0 = DBtoLIN(frame.Gain0)

	// Set coefficients of variable cascade resonators

	switch {
	case s.nfcascade >= 8:
		s.setABC(s.r8c, 7500, 600)
	case s.nfcascade >= 7:
		s.setABC(s.r7c, 6500, 500)
	case s.nfcascade >= 6:
		s.setABC(s.r6c, frame.F6hz, frame.B6hz)
	case s.nfcascade >= 5:
		s.setABC(s.r5c, frame.F5hz, frame.B5hz)
	}
	s.setABC(s.r4c, frame.F4hz, frame.B4hz)
	s.setABC(s.r3c, frame.F3hz, frame.B3hz)
	s.setABC(s.r2c, frame.F2hz, frame.B2hz)
	s.setABC(s.r1c, frame.F1hz, frame.B1hz)

	// Set coeficients of nasal resonator and zero antiresonator

	s.setABC(s.rnpc, frame.FNPhz, frame.BNPhz)
	s.setZeroABC(s.rnz, frame.FNZhz, frame.BNZhz)

	// Set coefficients of parallel resonators, and amplitude of outputs

	s.setABCG(s.r1p, frame.F1hz, frame.B1phz, amp_parF1)
	s.setABCG(s.rnpp, frame.FNPhz, frame.BNPhz, amp_parFNP)
	s.setABCG(s.r2p, frame.F2hz, frame.B2phz, amp_parF2)
	s.setABCG(s.r3p, frame.F3hz, frame.B3phz, amp_parF3)
	s.setABCG(s.r4p, frame.F4hz, frame.B4phz, amp_parF4)
	s.setABCG(s.r5p, frame.F5hz, frame.B5phz, amp_parF5)
	s.setABCG(s.r6p, frame.F6hz, frame.B6phz, amp_parF6)

	// output low-pass filter
	s.setABC(s.rout, 0, float64(s.sampleRate/2))
}

/*
	Generate a low pass filtered train of impulses as an approximation of
	a natural excitation waveform.
*/
func (s *Synthesizer) impulsiveSource() float64 {
	doublet := [...]float64{0, 13000000, -13000000}

	if s.nper < 3 {
		s.vwave = doublet[s.nper]
	} else {
		s.vwave = 0
	}
	/*
		Low-pass filter the differentiated impulse with a critically-damped
		second-order filter, time constant proportional to Kopen.
	*/
	return s.rgl.resonator(s.vwave)
}

/*
	Vwave is the differentiated glottal flow waveform.
	There is a weak spectral zero around 800 Hz.
	Magic constants a,b reset pitch synchronously.
*/
func (s *Synthesizer) naturalSource() float64 {
	// See if glottis open
	if s.nper >= s.nopen {
		// Glottis closed
		s.vwave = 0
		return 0
	}
	// Glottis open
	s.natglotA -= s.natglotB
	s.vwave += s.natglotA
	return s.vwave * 0.028
}

// Allows the use of a glottal excitation waveform sampled from a real voice.
func (s *Synthesizer) sampledSource() float64 {
	if s.t0 == 0 {
		return 0
	}

	ftemp := float64(s.nper) / float64(s.t0*len(s.naturalSamples))
	itemp := int(ftemp)
	temp_diff := ftemp - float64(itemp)

	current_value := s.naturalSamples[itemp]
	next_value := s.naturalSamples[itemp+1]

	diff_value := float64(next_value) - float64(current_value)
	diff_value *= temp_diff

	result := (float64(s.naturalSamples[itemp]) + diff_value) * s.sampleFactor
	return result
}

/*
	Adds F0 flutter, as specified in:

	"Analysis, synthesis and perception of voice quality variations among
	female and male talkers" D.H. Klatt and L.C. Klatt JASA 87(2) February 1990.

	Flutter is added by applying a quasi-random element constructed from three
	slowly varying sine waves.
*/
func (s *Synthesizer) flutter(frame *Frame) {
	timeCount := float64(s.flutterTimeCount)
	fla := s.f0Flutter / 50
	flb := s.originalF0 / 100
	flc := math.Sin(float64(2 * math.Pi * 12.7 * timeCount))
	fld := math.Sin(float64(2 * math.Pi * 7.1 * timeCount))
	fle := math.Sin(float64(2 * math.Pi * 4.7 * timeCount))
	delta_f0 := float64(fla*flb) * (flc + fld + fle) * 10
	frame.F0hz10 = frame.F0hz10 + int(delta_f0)
	s.flutterTimeCount++
}

// Convenience function for setting parallel resonators with gain
func (s *Synthesizer) setABCG(res *Resonator, f float64, bw float64, gain float64) {
	s.setABC(res, f, bw)
	res.a *= gain
}

// Convert formant freqencies and bandwidth into resonator difference equation coefficents.
// f: Frequency of resonator in Hz
// bw: Bandwidth of resonator in Hz
func (s *Synthesizer) setABC(res *Resonator, f float64, bw float64) {
	r := math.Exp(s.minusPiT * bw)       // Let r = exp(-pi bw t)
	res.c = -(r * r)                     // Let c = -r**2
	res.b = r * math.Cos(s.twoPiT*f) * 2 // Let b = r * 2*cos(2 pi f t)
	res.a = 1 - res.b - res.c            // Let a = 1.0 - b - c
}

// Convert formant freqencies and bandwidth into anti-resonator difference equation coefficents.
// f: Frequency of resonator in Hz
// bw: Bandwidth of resonator in Hz
func (s *Synthesizer) setZeroABC(res *Resonator, f float64, bw float64) {
	f = -f
	if f >= 0 {
		f = -1
	}

	s.setABC(res, f, bw) // First compute ordinary resonator coefficients

	// Now convert to antiresonator coefficients

	res.a = 1 / res.a // a'=  1/a
	res.c *= -res.a   // b'= -b/a
	res.b *= -res.a   // c'= -c/a
}

// Reset selected parameters pitch-synchronously.
func (s *Synthesizer) resetParams(frame *Frame) {
	if frame.F0hz10 <= 0 {
		s.t0 = 4 // Default for f0 undefined
		s.ampVoice = 0
		s.nmod = s.t0
		s.ampBreath = 0
		s.natglotA = 0
		s.natglotB = 0
	} else {
		// t0 is 4* the number of samples in one pitch period

		s.t0 = (40 * s.sampleRate) / frame.F0hz10
		// Period in samp*4
		s.ampVoice = DBtoLIN(frame.AVdb)

		// Duration of period before amplitude modulation

		s.nmod = s.t0
		if frame.AVdb > 0 {
			s.nmod /= 2
		}

		// Breathiness of voicing waveform

		s.ampBreath = DBtoLIN(frame.Aturb) * 0.1

		// Set open phase of glottal period where  40 <= open phase <= 263

		s.nopen = 4 * frame.Kopen

		if s.glsource == Impulsive && s.nopen > 263 {
			s.nopen = 263
		}

		if s.nopen >= s.t0-1 {
			s.nopen = s.t0 - 2
			if !s.QuietErrors {
				log.Println("Warning: glottal open period cannot exceed t0, truncated")
			}
		}

		if s.nopen < 40 {
			// F0 max = 1000 Hz
			s.nopen = 40
			if !s.QuietErrors {
				log.Println("Warning: minimum glottal open period is 10 samples.")
				log.Println("truncated, nopen = %v", s.nopen)
			}
		}

		// Reset a & b, which determine shape of "natural" glottal waveform

		s.natglotB = natglot[s.nopen-40]
		s.natglotA = (s.natglotB * float64(s.nopen)) * 0.333

		// Reset width of "impulsive" glottal pulse

		temp := s.sampleRate / s.nopen
		s.setABC(s.rgl, 0, float64(temp))

		// Make gain at F1 about constant

		temp1 := float64(s.nopen) * 0.00833
		s.rgl.a *= temp1 * temp1

		// Truncate skewness so as not to exceed duration of closed phase of glottal period.

		temp = s.t0 - s.nopen
		if frame.Kskew > temp {
			if !s.QuietErrors {
				log.Println("Kskew duration=%v > glottal closed period=%v, truncate",
					int(frame.Kskew), int(s.t0-s.nopen))
			}
			frame.Kskew = temp
		}

		if s.skew >= 0 {
			s.skew = frame.Kskew
		} else {
			s.skew = -frame.Kskew
		}

		// Add skewness to closed portion of voicing period

		s.t0 += s.skew
		s.skew = -s.skew
	}

	// Reset these parameters pitch-synchronously or at update rate if f0=0

	if s.t0 != 4 || s.ns == 0 {
		// Set one-pole low-pass filter that tilts glottal source
		s.decay = 0.033 * float64(frame.TLTdb)
		if s.decay > 0 {
			s.onemd = 1 - s.decay
		} else {
			s.onemd = 1
		}
	}
}

/*
 * Constant natglot[] controls shape of glottal pulse as a function
 * of desired duration of open phase N0
 * (Note that N0 is specified in terms of 40,000 samples/sec of speech)
 *
 *    Assume voicing waveform V(t) has form: k1 t**2 - k2 t**3
 *
 *    If the radiation characterivative, a temporal derivative
 *      is folded in, and we go from continuous time to discrete
 *      integers n:  dV/dt = vwave[n]
 *                         = sum over i=1,2,...,n of { a - (i * b) }
 *                         = a n  -  b/2 n**2
 *
 *      where the  constants a and b control the detailed shape
 *      and amplitude of the voicing waveform over the open
 *      potion of the voicing cycle "nopen".
 *
 *    Let integral of dV/dt have no net dc flow --> a = (b * nopen) / 3
 *
 *    Let maximum of dUg(n)/dn be constant --> b = gain / (nopen * nopen)
 *      meaning as nopen gets bigger, V has bigger peak proportional to n
 *
 *    Thus, to generate the table below for 40 <= nopen <= 263:
 *
 *      natglot[nopen - 40] = 1920000 / (nopen * nopen)
 */
var natglot = [...]float64{
	1200, 1142, 1088, 1038, 991, 948, 907, 869, 833, 799, 768, 738, 710, 683, 658,
	634, 612, 590, 570, 551, 533, 515, 499, 483, 468, 454, 440, 427, 415, 403,
	391, 380, 370, 360, 350, 341, 332, 323, 315, 307, 300, 292, 285, 278, 272,
	265, 259, 253, 247, 242, 237, 231, 226, 221, 217, 212, 208, 204, 199, 195,
	192, 188, 184, 180, 177, 174, 170, 167, 164, 161, 158, 155, 153, 150, 147,
	145, 142, 140, 137, 135, 133, 131, 128, 126, 124, 122, 120, 119, 117, 115,
	113, 111, 110, 108, 106, 105, 103, 102, 100, 99, 97, 96, 95, 93, 92, 91, 90,
	88, 87, 86, 85, 84, 83, 82, 80, 79, 78, 77, 76, 75, 75, 74, 73, 72, 71,
	70, 69, 68, 68, 67, 66, 65, 64, 64, 63, 62, 61, 61, 60, 59, 59, 58, 57,
	57, 56, 56, 55, 55, 54, 54, 53, 53, 52, 52, 51, 51, 50, 50, 49, 49, 48, 48,
	47, 47, 46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 41, 41, 41, 41, 40, 40,
	39, 39, 38, 38, 38, 38, 37, 37, 36, 36, 36, 36, 35, 35, 35, 35, 34, 34, 33,
	33, 33, 33, 32, 32, 32, 32, 31, 31, 31, 31, 30, 30, 30, 30, 29, 29, 29, 29,
	28, 28, 28, 28, 27, 27,
}

/*
 * Conversion table, db to linear, 87 dB --> 32767
 *                                 86 dB --> 29491 (1 dB down = 0.5**1/6)
 *                                 ...
 *                                 81 dB --> 16384 (6 dB down = 0.5)
 *                                 ...
 *                                  0 dB -->     0
 *
 * The just noticeable difference for a change in intensity of a vowel
 *   is approximately 1 dB.  Thus all amplitudes are quantized to 1 dB
 *   steps.
 */
func DBtoLIN(dB int) float64 {
	if dB < 0 || dB > 87 {
		return 0
	}
	return ampTable[dB] * 0.001
}

var ampTable = [...]float64{
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 7,
	8, 9, 10, 11, 13, 14, 16, 18, 20, 22, 25, 28, 32,
	35, 40, 45, 51, 57, 64, 71, 80, 90, 101, 114, 128,
	142, 159, 179, 202, 227, 256, 284, 318, 359, 405,
	455, 512, 568, 638, 719, 811, 911, 1024, 1137, 1276,
	1438, 1622, 1823, 2048, 2273, 2552, 2875, 3244, 3645,
	4096, 4547, 5104, 5751, 6488, 7291, 8192, 9093, 10207,
	11502, 12976, 14582, 16384, 18350, 20644, 23429,
	26214, 29491, 32767,
}
