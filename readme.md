
<h1 align="center"; style = font-size: 500px; > VroomAI </h1>

<h3 align="center">
	Generate sounds from words. Directly in your DAW.<br>
	Featured in Ableton's blog post, <a href="https://www.ableton.com/en/blog/ai-and-music-making-the-state-of-play/">AI and Music-Making</a>.<br>
	Checkout our latest project, <a href="https://github.com/vroomai/live">Vroom Live</a>.
</h3>

<br>
<p align="center">
  <img src="screenshot.png" alt="VroomAI"/>
</p>

## Summary
In the last few months the field of text-to-audio AI models has [rapidly evolved](https://github.com/archinetai/audio-ai-timeline). VroomAI is a VST plugin delivering the latest text-to-audio AI models directly to artists.

VroomAI was created as a submission to the [2023 Neural Audio Plugin Competition](https://www.theaudioprogrammer.com/neural-audio).

## Usage
Audio samples are saved into a user-specified directory and can be played at various pitches inside the DAW.

## Available Models
- [AudioLDM (Haohe Liu, Zehua Chen, Yi Yuan, Xinhao Mei, Xubo Liu, Danilo Mandic, Wenwu Wang, Mark D. Plumbley).](https://github.com/haoheliu/AudioLDM)
- *[MusicLM](https://google-research.github.io/seanet/musiclm/examples/) (Internal implementation coming soon)*

## Installation

At the moment the quickest way to get things working is to directly build the VST on your machine so we don't need to worry about code signing. A public inference server will be available shortly at vroomai.com but in the meantime you can host your own using [Haohe Liu's app.py](https://github.com/haoheliu/AudioLDM/blob/main/app.py) with the api enabled.

This plugin has only ever been tested on Reaper and Ableton on MacOS (M2).

## Discord

If you want to ask any questions or follow updates - [join the discord](https://discord.gg/Ua8sqvjher)!

## Authors
[Monty Anderson](https://montyanderson.net) ([Prodia Labs](https://prodia.com))

[Barney Hill](https://www.barneyhill.com) (University of Oxford)
