PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//1148935/1240919/2.50/5/2/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r115_60"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.6) (shapeHeight 1.15))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "SOT95P280X145-5N" (originalName "SOT95P280X145-5N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r115_60) (pt -1.3, 0.95) (rotation 90))
			(pad (padNum 2) (padStyleRef r115_60) (pt -1.3, 0) (rotation 90))
			(pad (padNum 3) (padStyleRef r115_60) (pt -1.3, -0.95) (rotation 90))
			(pad (padNum 4) (padStyleRef r115_60) (pt 1.3, -0.95) (rotation 90))
			(pad (padNum 5) (padStyleRef r115_60) (pt 1.3, 0.95) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.125 1.75) (pt 2.125 1.75) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.125 1.75) (pt 2.125 -1.75) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.125 -1.75) (pt -2.125 -1.75) (width 0.05))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -2.125 -1.75) (pt -2.125 1.75) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.812 1.45) (pt 0.812 1.45) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 0.812 1.45) (pt 0.812 -1.45) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 0.812 -1.45) (pt -0.812 -1.45) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.812 -1.45) (pt -0.812 1.45) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.812 0.5) (pt 0.138 1.45) (width 0.025))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.375 1.45) (pt 0.375 1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0.375 1.45) (pt 0.375 -1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0.375 -1.45) (pt -0.375 -1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.375 -1.45) (pt -0.375 1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.875 1.5) (pt -0.725 1.5) (width 0.2))
		)
	)
	(symbolDef "TS3021AILT" (originalName "TS3021AILT")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 1100 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 1100 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 900 mils 100 mils) (width 6 mils))
		(line (pt 900 mils 100 mils) (pt 900 mils -300 mils) (width 6 mils))
		(line (pt 900 mils -300 mils) (pt 200 mils -300 mils) (width 6 mils))
		(line (pt 200 mils -300 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 950 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 950 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "TS3021AILT" (originalName "TS3021AILT") (compHeader (numPins 5) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "OUT") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "VCC-") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "IN+") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "IN-") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "VCC+") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "TS3021AILT"))
		(attachedPattern (patternNum 1) (patternName "SOT95P280X145-5N")
			(numPads 5)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
			)
		)
		(attr "Mouser Part Number" "511-TS3021AILT")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/STMicroelectronics/TS3021AILT?qs=U58vgkJzipTYTZ89iH%2FIVA%3D%3D")
		(attr "Manufacturer_Name" "STMicroelectronics")
		(attr "Manufacturer_Part_Number" "TS3021AILT")
		(attr "Description" "Rail-to-rail 1.8 V high-speed comparator, small input offset voltage")
		(attr "<Hyperlink>" "https://www.st.com/resource/en/datasheet/ts3021.pdf")
		(attr "<Component Height>" "1.45")
		(attr "<STEP Filename>" "TS3021AILT.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)
