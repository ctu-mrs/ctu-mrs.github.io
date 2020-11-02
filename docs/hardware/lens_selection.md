# A quickstart guide to camera and lens selection for computer vision

This guide will shortly explain how to select a camera + lens for a computer vision application.
*This guide assumes that the optical system may be modelled as a [pinhole camera](https://en.wikipedia.org/wiki/Pinhole_camera_model) and the process and the equations get a tad more complicated if this model cannot be used* (ie. when you intend to use special wide-FOV/fisheye/omnidirectional optics).
Still, it should give you a good overview of the problematic.

In most cases, you want to select an optical imaging system (camera + lens in other words) to comply with one of the following combinations of parameters:

 A) image resolution, field of view (FOV),
 B) pixel size of an object with a defined size at a specific distance,
 C) image resolution, maximal distortion of the image (how far is the image from an ideal pinhole camera).

Additionally, you usually want to ensure a minimal frame rate and sometimes a specific connection option is required (USB2, USB3, Ethernet etc.).
Two useful conversion equations are:

 1) <img src="https://render.githubusercontent.com/render/math?math=r_o=\frac{rs_of}{s(d-f)}">,

where <img src="https://render.githubusercontent.com/render/math?math=r_o"> is pixel size of an object with size <img src="https://render.githubusercontent.com/render/math?math=s_o"> at distance <img src="https://render.githubusercontent.com/render/math?math=d">, <img src="https://render.githubusercontent.com/render/math?math=s"> is size of the sensor, and <img src="https://render.githubusercontent.com/render/math?math=f"> is focal distance of the camera.

 2) <img src="https://render.githubusercontent.com/render/math?math=FOV=2\mathrm{atan}\left(\frac{s}{2f}\right)">,

where <img src="https://render.githubusercontent.com/render/math?math=FOV"> is an angular field of view of the optical imaging system.

These equations can be used to convert between cases A) and B).

OK, hele tak výběr objektivu závisí hlavně na třech parametrech kamery: Typ držáku objektivu (Lens Mount), velikost senzoru kamery (Sensor Format) a rozlišení senzoru (Resolution).


	
Lens Mount: Omezuje, jaký objektiv vůbec lze namontovat. Jsou tři základní typy: C, CS a M12. Objektiv typu C lze většinou bez problému použít pro kameru s CS držákem při použití redukce. Objektiv typu CS nelze použít pro kameru s C držákem. Existují také redukce pro použití M12 objektivu pro C/CS držák, ale tato kombinace bývá problematická. Další info: https://www.flir.com/support-center/iis/machine-vision/application-note/selecting-a-lens-for-your-camera/
	Sensor Format: Nejsnazší je vybrat optiku, určenou pro konkrétní velikost senzoru. Lze použít i optiku, určenou pro jinou velikost senzoru, ale můsíš potom přepočítat ostatní parametry optiky. Pokud použiješ optiku, určenou pro menší velikost senzoru, tak se ti pravděpodobně stane to, že světlo bude skrz optiku dopadat jen na menší část senzoru (odpovídající rozměru, pro který je optika určena), takže bude obraz oříznutý, a nebo bude na okrajích výrazně zkreslený. Taky bude potřeba přepočítat ostatní parametry objektivu, pokud ho použiješ pro jinak velký senzor.
	Resolution: Kvalita objektivu by měla odpovídat rozlišení senzoru. Pokud použiješ 10Mpx senzor a nekvalitní optiku, tak ti těch 10Mpx bude k ničemu. Správně by prodejce/výrobce objektivu měl vždy uvádět pro jaké rozlišení senzoru je objektiv určen.

