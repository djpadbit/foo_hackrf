#include "stdafx.h"
#include "resource.h"
#include <libPPUI/wtl-pp.h> // CCheckBox
#include <helpers/DarkMode.h>
#include <hackrf.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <mutex> 
#include <vector>

#define BUF_LEN 262144         //hackrf tx buf
#define BUF_NUM  32
#define M_PI 3.14159265358979323846
#define SAMPLERATE 2850000

DECLARE_COMPONENT_VERSION(
"HackRF Transmitter",
"0.1.1.0",
"Code: https://github.com/djpadbit/foo_hackrf \n"
"DLL: https://github.com/djpadbit/foo_hackrf/releases");

// This will prevent users from renaming your component around (important for proper troubleshooter behaviors) or loading multiple instances of it.
VALIDATE_COMPONENT_FILENAME("foo_hackrf.dll");

// Activate cfg_var downgrade functionality if enabled. Relevant only when cycling from newer FOOBAR2000_TARGET_VERSION to older.
FOOBAR2000_IMPLEMENT_CFG_VAR_DOWNGRADE;


struct config {
	float freq;
	uint32_t gain;
	uint32_t mode;
	uint32_t tx_vga;
	uint32_t enableamp;
	uint32_t enstereo;
	uint32_t tau;

};
static config default{
	433.00,//433Mhz
		90,
		0,//mode WBFM=0 NBFM=1 AM=2
		40,
		0,
		1,
		0}; //tau 50us=0 75us=1 0us=2

static void RunDSPConfigPopup(const dsp_preset & p_data, HWND p_parent, dsp_preset_edit_callback & p_callback);

int _hackrf_tx_callback(hackrf_transfer * transfer);

class dsp_hackrf : public dsp_impl_base_t<dsp_v3>
{
public:

	int hackrf_tx_callback(int8_t* buffer, uint32_t length) {

		std::unique_lock<std::mutex> lock(mutex);

		if (count == 0) {
			memset(buffer, 0, length);
		}
		else {
			memcpy(buffer, _buf[tail], length);
			tail = (tail + 1) % BUF_NUM;
			count--;
		}

		cond.notify_one();

		return 0;
	}

	void set_config(config& config) {
		freq = uint64_t(config.freq * 1000000);
		dsp_gain = (float)(config.gain / 100.0);
		mode = config.mode;
		tx_vga = config.tx_vga;
		enableamp = config.enableamp;
		enstereo = config.enstereo;
		tau = config.tau;
	}

	dsp_hackrf(dsp_preset const& in) : conf(default) {
		parse_preset(conf, in);
		set_config(conf);

		fm_phase = 0.0f;
		count = tail = head = offset = 0;
		samp_avail = BUF_LEN / 2;
		_buf = (int8_t**)malloc(BUF_NUM * sizeof(int8_t*));
		if (!_buf)
			throw std::runtime_error("Failed to allocated memory");
		for (unsigned int i = 0; i < BUF_NUM; ++i) {
			_buf[i] = (int8_t*)malloc(BUF_LEN * sizeof(int8_t));
			if (!_buf[i])
				throw std::runtime_error("Failed to allocated memory");
		}

		// hackrf init  //
		hackrf_init();
		ret = hackrf_open(&_dev);
		if (ret != HACKRF_SUCCESS) {
			hackrf_close(_dev);
			throw std::runtime_error("Failed to open HackRF device");
		}

		hackrf_set_sample_rate(_dev, SAMPLERATE);
		hackrf_set_baseband_filter_bandwidth(_dev, SAMPLERATE * 3 / 4);
		hackrf_set_freq(_dev, freq);
		hackrf_set_txvga_gain(_dev, tx_vga);
		hackrf_set_lna_gain(_dev, 24);
		hackrf_set_amp_enable(_dev, enableamp);
		ret = hackrf_start_tx(_dev, _hackrf_tx_callback, (void*)this);
		if (ret != HACKRF_SUCCESS) {
			hackrf_close(_dev);
			throw std::runtime_error("Failed to start TX streaming");
		}
	}

	~dsp_hackrf() {
		if (_dev) {
			hackrf_stop_tx(_dev);
			hackrf_close(_dev);
			_dev = nullptr;
		}
		if (_buf) {
			for (unsigned int i = 0; i < BUF_NUM; ++i) {
				if (_buf[i])
					free(_buf[i]);
			}
			free(_buf);
		}
	}

	static GUID g_get_guid() {
		//This is our GUID. Generate your own one when reusing this code.
		// {67BB9226-3830-4739-BD72-3951BF207388}
		static const GUID guid = { 0x67bb9226, 0x3830, 0x4739,{ 0xbd, 0x72, 0x39, 0x51, 0xbf, 0x20, 0x73, 0x88 } };

		return guid;
	}

	static void g_get_name(pfc::string_base& p_out) { p_out = "HackRF Transmitter"; }

	bool on_chunk(audio_chunk* chunk, abort_callback&) {
		// Perform any operations on the chunk here.
		// The most simple DSPs can just alter the chunk in-place here and skip the following functions.
		audio_sample* source_audio_buf = chunk->get_data();
		sample_count = chunk->get_sample_count();

		integerfactor = SAMPLERATE * 1.0 / chunk->get_sample_rate();

		if (input_audio_buf.first.size() != sample_count)
			input_audio_buf.first.resize(sample_count);
		if (input_audio_buf.second.size() != sample_count)
			input_audio_buf.second.resize(sample_count);

		if (mixed_output_audio_buf.size() != size_t(sample_count * integerfactor)) {
			output_audio_buf.first.resize(size_t(sample_count * integerfactor));
			output_audio_buf.second.resize(size_t(sample_count * integerfactor));

			mixed_output_audio_buf.resize(size_t(sample_count * integerfactor));

			iq_buf.resize(size_t(sample_count * integerfactor) * 2);
		}

		if (chunk->get_channels() == 1 && chunk->get_channel_config() == audio_chunk::channel_config_mono) {
			for (uint32_t i = 0; i < sample_count; i++) {
				input_audio_buf.first[i] = source_audio_buf[i];
			}
		}
		else if (chunk->get_channels() == 2 && chunk->get_channel_config() == audio_chunk::channel_config_stereo) { /* Stereo */
			for (uint32_t i = 0; i < sample_count; i++) {
				input_audio_buf.first[i] = source_audio_buf[i * 2];
				input_audio_buf.second[i] = source_audio_buf[i * 2 + 1];
			}
		}

		interpolation(input_audio_buf.first.data(), sample_count, output_audio_buf.first.data(), (uint32_t)(sample_count * integerfactor), last_in_samples_r);
		if ((mode != 2) && (tau != 2)) /* FM mode */
			preemph(output_audio_buf.first.data(), prev_samples[0], (uint32_t)(sample_count * integerfactor), tau);

		if (chunk->get_channels() == 1 && chunk->get_channel_config() == audio_chunk::channel_config_mono) {
			mixed_output_audio_buf.swap(output_audio_buf.first);
		} else {
			interpolation(input_audio_buf.second.data(), sample_count, output_audio_buf.second.data(), (uint32_t)(sample_count * integerfactor), last_in_samples_l);
			if ((mode != 2) && (tau != 2)) /* FM mode */
				preemph(output_audio_buf.second.data(), prev_samples[1], (uint32_t)(sample_count * integerfactor), tau);

			if (enstereo && (!mode)) { /* Stereo is available only to WBFM mode */
				for (uint32_t i = 0; i < (size_t)(sample_count * integerfactor); i++) {
					mixed_output_audio_buf[i] = float(((output_audio_buf.first[i] + output_audio_buf.second[i]) * 0.5 + \
						(output_audio_buf.first[i] - output_audio_buf.second[i]) * 0.5 * sin(19000 * 4 * M_PI * (double(duration) / double(SAMPLERATE)))) * 0.9 + \
						sin(19000 * 2 * M_PI * (double(duration) / double(SAMPLERATE))) * 0.1);
					//mixed_output_audio_buf[i] = float(sin(19000 * 4 * M_PI * (duration / double(SAMPLERATE)))*0.9 + sin(19000 * 2 * M_PI * (duration / double(SAMPLERATE))) * 0.1);

					if ((++duration) >= (4 * SAMPLERATE / 19000))
						duration = 0;
				}
			}
			else {
				for (uint32_t i = 0; i < (size_t)(sample_count * integerfactor); i++) {
					mixed_output_audio_buf[i] = float((output_audio_buf.second[i] + output_audio_buf.first[i]) * 0.5f);
				}
			}
		}

		modulation(mixed_output_audio_buf.data(), size_t(sample_count * integerfactor), iq_buf.data(), mode);

		send(iq_buf.data(), size_t(sample_count * integerfactor));


		// To retrieve the currently processed track, use get_cur_file().
		// Warning: the track is not always known - it's up to the calling component to provide this data and in some situations we'll be working with data that doesn't originate from an audio file.
		// If you rely on get_cur_file(), you should change need_track_change_mark() to return true to get accurate information when advancing between tracks.

		return true; //Return true to keep the chunk or false to drop it from the chain.
	}

	void on_endofplayback(abort_callback&) {
		// The end of playlist has been reached, we've already received the last decoded audio chunk.
		// We need to finish any pending processing and output any buffered data through insert_chunk().
	}
	void on_endoftrack(abort_callback&) {
		// Should do nothing except for special cases where your DSP performs special operations when changing tracks.
		// If this function does anything, you must change need_track_change_mark() to return true.
		// If you have pending audio data that you wish to output, you can use insert_chunk() to do so.		
	}

	void flush() {
		// If you have any audio data buffered, you should drop it immediately and reset the DSP to a freshly initialized state.
		// Called after a seek etc.
	}

	double get_latency() {
		// If the DSP buffers some amount of audio data, it should return the duration of buffered data (in seconds) here.
		return 0;
	}

	bool need_track_change_mark() {
		// Return true if you need on_endoftrack() or need to accurately know which track we're currently processing
		// WARNING: If you return true, the DSP manager will fire on_endofplayback() at DSPs that are before us in the chain on track change to ensure that we get an accurate mark, so use it only when needed.
		return false;
	}

	bool apply_preset(const dsp_preset& preset) {
		// Parse the preset
		config cfg;
		parse_preset(cfg, preset);

		// Change hackrf settings
		if (cfg.freq != conf.freq)
			hackrf_set_freq(_dev, uint64_t(cfg.freq * 1000000));
		
		if (cfg.tx_vga != conf.tx_vga)
			hackrf_set_txvga_gain(_dev, cfg.tx_vga);
		
		if (cfg.enableamp != conf.enableamp)
			hackrf_set_amp_enable(_dev, cfg.enableamp);

		// Apply the change to the current settings
		conf = cfg;
		set_config(conf);

		return true;
	}

	static bool g_get_default_preset(dsp_preset& p_out) {
		make_preset(default, p_out);
		return true;
	}

	static void g_show_config_popup(const dsp_preset& p_data, HWND p_parent, dsp_preset_edit_callback& p_callback) {
		RunDSPConfigPopup(p_data, p_parent, p_callback);
	}

	static bool g_have_config_popup() { return true; }

	static void make_preset(config conf, dsp_preset& out) {
		dsp_preset_builder builder;
		builder << conf.freq << conf.gain << conf.mode << conf.tx_vga << conf.enableamp << conf.enstereo << conf.tau;
		builder.finish(g_get_guid(), out);
	}

	static void parse_preset(config& conf, const dsp_preset& in) {
		try {
			dsp_preset_parser parser(in);
			parser >> conf.freq >> conf.gain >> conf.mode >> conf.tx_vga >> conf.enableamp >> conf.enstereo >> conf.tau;
		}
		catch (exception_io_data) { conf = default; }
	}

private:

	void interpolation(float* in_buf, uint32_t in_samples, float* out_buf, uint32_t out_samples, float last_in_samples[4]) {
		uint32_t i;		/* Input buffer index + 1. */
		uint32_t j = 0;	/* Output buffer index. */
		float pos;		/* Position relative to the input buffer
						* + 1.0. */

						/* We always "stay one sample behind", so what would be our first sample
						* should be the last one wrote by the previous call. */
		pos = (float)in_samples / (float)out_samples;
		while (pos < 1.0) {
			out_buf[j] = last_in_samples[3] + (in_buf[0] - last_in_samples[3]) * pos;
			j++;
			pos = (float)(j + 1) * (float)in_samples / (float)out_samples;
		}

		/* Interpolation cycle. */
		i = (uint32_t)pos;
		while (j < (out_samples - 1)) {
			out_buf[j] = in_buf[i - 1] + (in_buf[i] - in_buf[i - 1]) * (pos - (float)i);
			j++;
			pos = (float)(j + 1) * (float)in_samples / (float)out_samples;
			i = (uint32_t)pos;
		}

		/* The last sample is always the same in input and output buffers. */
		out_buf[j] = in_buf[in_samples - 1];

		/* Copy last samples to last_in_samples (reusing i and j). */
		for (i = in_samples - 4, j = 0; j < 4; i++, j++)
			last_in_samples[j] = in_buf[i];
	}

	void preemph(float* buf, float prev[2], uint32_t samples, uint32_t tau_75) {
		static const double preemph_btaps[2][2] = { 29.498236311937575, -27.70989987735248, 43.80803296614535, -42.01969653156026 };
		static const double preemph_ataps[2] = { 1.0, 0.7883364345850924 };

		float last_input;

		for (uint32_t i = 0; i < samples; i++) {
			last_input = prev[0];
			prev[0] = buf[i];

			/* b0x(n) + b1x(n - 1) - a1 * y(n - 1) */
			buf[i] = (float)(preemph_btaps[tau_75][0] * buf[i] + preemph_btaps[tau_75][1] * last_input - preemph_ataps[1] * prev[1]);

			prev[1] = buf[i];
		}

	}

	void modulation(float* input, uint32_t input_len, float* output, uint32_t mode) {
		if (mode == 0)
			fm_deviation = 2.0 * M_PI * 75.0e3 / SAMPLERATE; // 75 kHz max deviation WBFM
		else if (mode == 1)
			fm_deviation = 2.0 * M_PI * 5.0e3 / SAMPLERATE; // 5 kHz max deviation NBFM

		if (mode == 2) { //AM mode
			for (uint32_t i = 0; i < input_len; i++) {
				double	audio_amp = input[i] * dsp_gain + 0.5;

				if (fabs(audio_amp) > 1.0)
					audio_amp = (audio_amp > 0.0) ? 1.0 : -1.0;

				output[i * 2] = (float)audio_amp;
				output[i * 2 + 1] = 0;
			}
		} else { //FM mode
			for (uint32_t i = 0; i < input_len; i++) {
				double	audio_amp = input[i] * dsp_gain;

				if (fabs(audio_amp) > 1.0)
					audio_amp = (audio_amp > 0.0) ? 1.0 : -1.0;

				fm_phase += fm_deviation * audio_amp;
				while (fm_phase > M_PI)
					fm_phase -= 2.0 * M_PI;
				while (fm_phase < -M_PI)
					fm_phase += 2.0 * M_PI;

				output[i * 2] = (float)sin(fm_phase);
				output[i * 2 + 1] = (float)cos(fm_phase);
			}
		}
	}

	void send(float* input_items, size_t len) {
		{
			std::unique_lock <std::mutex> lock(mutex);
			while (count == BUF_NUM)
				cond.wait_for(lock, std::chrono::microseconds(1000));
		}

		size_t proc_len = 0;

		while (proc_len < len) {
			size_t canput = min(len - proc_len, samp_avail);
			int8_t* buf = (int8_t*)_buf[head] + offset * 2;
			for (uint32_t i = 0; i < canput; i++) {
				buf[i * 2] = (int8_t)(input_items[i * 2 + proc_len * 2] * 127.0);
				buf[i * 2 + 1] = (int8_t)(input_items[i * 2 + proc_len * 2 + 1] * 127.0);
			}

			proc_len += canput;
			offset += canput;
			samp_avail -= canput;
			if (!samp_avail) {
				{
					std::unique_lock <std::mutex> lock(mutex);
					head = (head + 1) % BUF_NUM;
					count++;
				}
				offset = 0;
				samp_avail = BUF_LEN / 2;
			}
		}
	}

	std::pair<std::vector<float>, std::vector<float> > input_audio_buf, output_audio_buf;
	std::vector<float> mixed_output_audio_buf;
	std::vector<float> iq_buf;
	std::condition_variable cond;
	std::mutex mutex;
	hackrf_device* _dev = nullptr;
	int ret;
	uint64_t freq;
	float dsp_gain;
	uint32_t mode;
	uint32_t tx_vga;
	uint8_t enableamp;
	uint32_t enstereo;
	uint32_t tau;

	double integerfactor;
	size_t sample_count;
	uint64_t duration = 0;

	config conf;
	int8_t** _buf = nullptr;
	int count;
	int tail;
	int head;
	int offset;
	uint32_t samp_avail;

	double fm_phase;
	double fm_deviation;
	float last_in_samples_l[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	float last_in_samples_r[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

	float prev_samples[2][2] = { 0.0f, 0.0f, 0.0f, 0.0f };
};

int _hackrf_tx_callback(hackrf_transfer * transfer) {
	dsp_hackrf* obj = (dsp_hackrf*)transfer->tx_ctx;
	return obj->hackrf_tx_callback((int8_t*)transfer->buffer, transfer->valid_length);
}

// Use dsp_factory_nopreset_t<> instead of dsp_factory_t<> if your DSP does not provide preset/configuration functionality.
static dsp_factory_t<dsp_hackrf> g_dsp_hackrf_factory;

class CMyDSPPopup : public CDialogImpl<CMyDSPPopup> {
public:
	CMyDSPPopup(const dsp_preset& initData, dsp_preset_edit_callback& callback) : m_initData(initData), m_callback(callback) {}

	enum { IDD = IDD_DSP };
	enum {
		GainMin = 0,
		GainMax = 100,
		GainTotal = GainMax - GainMin,
		TXGainMin = 0,
		TXGainMax = 47,
		TxGainTotal = TXGainMax - TXGainMin
	};

	BEGIN_MSG_MAP(CMyDSPPopup)
		MSG_WM_INITDIALOG(OnInitDialog)
		MSG_WM_HSCROLL(OnHScroll)
		MSG_WM_COMMAND(OnCommand)
	END_MSG_MAP()

private:

	BOOL OnInitDialog(CWindow, LPARAM) {
		m_dark.AddDialogWithControls(m_hWnd);
		config _config;
		m_slider = GetDlgItem(IDC_SLIDER);
		m_slider_tx = GetDlgItem(IDC_SLIDER_TX);
		m_edit_freq = GetDlgItem(IDC_EDIT_FREQ);
		m_check_amp = GetDlgItem(IDC_CHECK_AMP);
		m_check_stereo = GetDlgItem(IDC_CHECK_STEREO);
		m_combo_mode = GetDlgItem(IDC_COMBO_MODE);
		m_combo_tau = GetDlgItem(IDC_COMBO_TAU);
		m_slider.SetRange(0, GainTotal);
		m_slider_tx.SetRange(0, TxGainTotal);
		{
			dsp_hackrf::parse_preset(_config, m_initData);
			m_slider.SetPos(pfc::clip_t<t_int32>(pfc::rint32(_config.gain), GainMin, GainMax) - GainMin);
			m_slider_tx.SetPos(pfc::clip_t<t_int32>(pfc::rint32(_config.tx_vga), TXGainMin, TXGainMax) - TXGainMin);

			m_edit_freq.SetLimitText(7);
			char freq[20];
			sprintf_s(freq, "%.2f", _config.freq);
			wchar_t wstr[20];
			size_t ret_val;
			mbstowcs_s(&ret_val, wstr, freq, 20);
			m_edit_freq.SetWindowTextW(wstr);

			m_check_amp.SetCheck(_config.enableamp);
			m_check_stereo.SetCheck(_config.enstereo);

			m_combo_mode.AddString(L"WBFM");
			m_combo_mode.AddString(L"NBFM");
			m_combo_mode.AddString(L"AM");
			m_combo_mode.SetCurSel(_config.mode);

			// 75us premph is mostly used in the US
			// while 50us is mostly in Europe
			m_combo_tau.AddString(L"50μs");
			m_combo_tau.AddString(L"75μs");
			m_combo_tau.AddString(L"0μs");
			m_combo_tau.SetCurSel(_config.tau);

			RefreshLabel((uint32_t)_config.gain);
			RefreshTXLabel(_config.tx_vga);

			if (m_combo_mode.GetCurSel() != 0) {
				m_check_stereo.EnableWindow(FALSE);
				if (m_combo_mode.GetCurSel() == 2)
					m_combo_tau.EnableWindow(FALSE);
			}

			conf = _config;
		}
		return TRUE;
	}

	void UpdatePreset() {
		CString str;
		m_edit_freq.GetWindowTextW(str);
		conf.freq = static_cast<float>(_wtof(str));

		conf.mode = m_combo_mode.GetCurSel();
		conf.tau = m_combo_tau.GetCurSel();
		conf.enableamp = m_check_amp.GetCheck();
		conf.enstereo = m_check_stereo.GetCheck();
		conf.gain = (uint32_t)m_slider.GetPos();
		conf.tx_vga = (uint32_t)m_slider_tx.GetPos();

		dsp_preset_impl preset;
		dsp_hackrf::make_preset(conf, preset);
		m_callback.on_preset_changed(preset);
	}

	void OnCommand(UINT uNotifyCode, int nID, CWindow wndCtl) {
		switch (nID) {
			case IDAPPLY:
				UpdatePreset();
				break;
			case IDOK:
				UpdatePreset();
				EndDialog(IDOK);
				break;
			case IDCANCEL:
				EndDialog(0);
				break;
			case IDC_COMBO_MODE:
				if (uNotifyCode == CBN_SELCHANGE) {
					switch (m_combo_mode.GetCurSel()) {
						case 0: /* WBFM mode */
							m_check_stereo.SetCheck(1);
							m_check_stereo.EnableWindow();

							m_combo_tau.SetCurSel(0);
							m_combo_tau.EnableWindow();
							break;
						case 1: /* NBFM mode */
							m_check_stereo.SetCheck(0);
							m_check_stereo.EnableWindow(FALSE);

							m_combo_tau.SetCurSel(0);
							m_combo_tau.EnableWindow();
							break;
						case 2: /* AM mode */
							m_check_stereo.SetCheck(0);
							m_check_stereo.EnableWindow(FALSE);

							m_combo_tau.SetCurSel(2);
							m_combo_tau.EnableWindow(FALSE);
							break;
					}
					m_check_stereo.UpdateWindow();
					m_combo_tau.UpdateWindow();
				}
				break;
		}
	}

	void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar pScrollBar) {
		conf.gain = (uint32_t)m_slider.GetPos();
		conf.tx_vga = (uint32_t)m_slider_tx.GetPos();

		RefreshLabel((uint32_t)conf.gain);
		RefreshTXLabel(conf.tx_vga);
	}

	void RefreshTXLabel(uint32_t val) {
		pfc::string_formatter msg; msg << pfc::format_int(val);
		::uSetDlgItemText(*this, IDC_SLIDER_TX_LABEL, msg);
	}

	void RefreshLabel(uint32_t val) {
		pfc::string_formatter msg; msg << pfc::format_int(val);
		::uSetDlgItemText(*this, IDC_SLIDER_LABEL, msg);
	}

	const dsp_preset& m_initData; // modal dialog so we can reference this caller-owned object.
	dsp_preset_edit_callback& m_callback;

	CEdit m_edit_freq;
	CTrackBarCtrl m_slider;
	CTrackBarCtrl m_slider_tx;
	CCheckBox m_check_amp;
	CCheckBox m_check_stereo;
	CComboBox m_combo_mode;
	CComboBox m_combo_tau;
	config conf;
	fb2k::CDarkModeHooks m_dark;
};

static void RunDSPConfigPopup(const dsp_preset & p_data, HWND p_parent, dsp_preset_edit_callback & p_callback) {
	CMyDSPPopup popup(p_data, p_callback);
	if (popup.DoModal(p_parent) != IDOK) {
		// If the dialog exited with something else than IDOK,k 
		// tell host that the editing has been cancelled by sending it old preset data that we got initialized with
		p_callback.on_preset_changed(p_data);
	}
}
