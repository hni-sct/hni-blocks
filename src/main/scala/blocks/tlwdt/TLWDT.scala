package hni.blocks.wdt

import hni.blocks.devices.watchdog._

import Chisel._
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.subsystem.BaseSubsystem
import freechips.rocketchip.subsystem._
import freechips.rocketchip.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.util._
import chisel3.experimental.withClock

case class WDTParams(address: BigInt = 0x2000, regBytes: Int = 4 , useAXI4: Boolean = false,
  Dogs : Int = 4, Resets : Int = 4, Ints : Int = 3, 
  Mode : hniWatchdogTimer.Modes = hniWatchdogTimer.both, countWidth : Int = 31, cmpWidth : Int = 16, PulseWidth : Int = 32,
  PRBS : Boolean = false, PRBS_Set : Set[Int] = Set(31,28), Key : Int = 0x51F15E) 
{
  def Offset:Int = 0
}

case object WDTKey extends Field[Option[WDTParams]](None)

case object WDTListKey extends Field[Option[Seq[WDTParams]]](None)

class WDIO(Resets : Int) extends Bundle {
  val outputs = (Vec(Resets,Bool(OUTPUT)))
  val clock = Clock(INPUT)
}

trait WDTBundle
{
  val params: WDTParams
  val c = params

  val wdog = new WDIO(c.Dogs)
}

trait WDTModule extends HasRegMap
{
  val params: WDTParams
  val io: WDTBundle
  val interrupts: Vec[Bool]
  val c = params

  // We use external Clock -> Connect externally to the Clock you want
  withClock(io.wdog.clock){
    val wdog = Module(new WatchdogArray(Dogs = c.Dogs, Resets = c.Resets, Ints = c.Ints, Mode= c.Mode, PulseWidth = c.PulseWidth, Offset = c.Offset, 
                    PRBS = c.PRBS, PRBS_Set = c.PRBS_Set, Key = c.Key, 
                    regWidth = c.regBytes*8, countWidth = c.countWidth, cmpWidth = c.cmpWidth))
    io.wdog.outputs := wdog.io.outputs
    wdog.io.corerst := false.B  // has to be false or true???
    for(i<-0 until c.Ints){
      interrupts := wdog.io.interrupts
    }
    regmap(
      (WatchdogArray.arrRegMap(wdog, c.Offset, c.regBytes, c.Dogs) :_*)
    ) 
  }
}

// Create a concrete TL2 version of the abstract Example slave
class TLWDT( params: WDTParams, beatBytes:Int)(implicit p: Parameters)
  extends TLRegisterRouter(
  params.address,
  "wdogarray", 
  Seq("hni,WatchdogArray"), 
  beatBytes = beatBytes,
  interrupts = params.Ints,
  concurrency = 1)(
  new TLRegBundle(params, _)    with WDTBundle)(
  new TLRegModule(params, _, _) with WDTModule) 

class AXI4WDT(params: WDTParams, beatBytes: Int)(implicit p: Parameters)
  extends AXI4RegisterRouter(
    params.address,
    beatBytes=beatBytes,
    interrupts = params.Ints,
    concurrency = 1)(
      new AXI4RegBundle(params, _) with WDTBundle)(
      new AXI4RegModule(params, _, _) with WDTModule)

// java -jar rocket-chip/sbt-launch.jar ++2.12.4 "runMain freechips.rocketchip.system.Generator /home/julian/RISCV/freedom-e/builds/e300artydevkit sifive.freedom.everywhere.e300artydevkit E300ArtyDevKitFPGAChip sifive.freedom.everywhere.e300artydevkit E300ArtyDevKitConfig"
trait CanHavePeripheryWDT 
{ this: BaseSubsystem =>
  private val portName = "wdogarray"

  val crossing = LazyModule(new TLAsyncCrossingSink(AsyncQueueParams.singleton()))
  var tlaxi4 : Option[TLToAXI4] = None
  val wdt = p(WDTKey) match {
    case Some(params) => {
      if (params.useAXI4) {
        tlaxi4 = Some(LazyModule(new TLToAXI4()))
        val wdt = LazyModule(new AXI4WDT(params, pbus.beatBytes)(p))
        pbus.toSlave(Some(portName)) {
          wdt.node :=
          AXI4Buffer() :=
          tlaxi4.get.node :=
          crossing.node :=
          TLAsyncCrossingSource() :=
          // toVariableWidthSlave doesn't use holdFirstDeny, which TLToAXI4() needsx
          TLFragmenter(pbus.beatBytes, pbus.blockBytes, holdFirstDeny = true)
        }
        ibus.fromSync := IntSyncCrossingSink() := IntSyncCrossingSource(alreadyRegistered = true)  := wdt.intnode // Ibus -> Connecting to Interruptsystem
        Some(wdt)
      } else {
        val wdt = LazyModule(new TLWDT(params, pbus.beatBytes)(p))
        val node = wdt.node := crossing.node 
        pbus.toVariableWidthSlave(Some(portName)) { node := TLAsyncCrossingSource() }  // Pbus -> Connecting to PeripherialBus // sbus -> Systembus
        ibus.fromSync := IntSyncCrossingSink() := IntSyncCrossingSource(alreadyRegistered = true)  := wdt.intnode // Ibus -> Connecting to Interruptsystem
        Some(wdt)
      }
    }
    case None => None
  }
}

trait CanHavePeripheryWDTList
{ this: BaseSubsystem =>
  
  val wdt = p(WDTListKey) match {
    case Some(params) => {
      Some(
        params.map{ ps =>
          TLWDT.attach(ps, p, pbus, ibus)
      })
    }
    case None => None
  }
}

trait CanHavePeripheryWDTModuleImp extends LazyModuleImp {
  val outer: CanHavePeripheryWDT
  
  val wdt_io = outer.wdt match {
    case Some(wdt) => {
      val c = wdt.module.params
      val wdog = IO(new WDIO( c.Resets ))
      wdog.outputs := wdt.module.io.wdog.outputs
      wdt.module.io.wdog.clock := wdog.clock
      outer.crossing.module.clock := wdog.clock
      if(outer.tlaxi4 != None){
        outer.tlaxi4.get.module.clock := wdog.clock
      }
      Some(wdog)
    }
    case None => None
  }
}

trait CanHavePeripheryWDTListModuleImp extends LazyModuleImp {
  val outer: CanHavePeripheryWDTList
  
  val wdt_io = outer.wdt match {
    case Some(wdt) => { 
      Some(wdt.map{ case (tmod: Either[TLWDT, (AXI4WDT, TLToAXI4)], crossing: TLAsyncCrossingSink) =>
        tmod match{
          case Right((mod, toaxi4)) =>{
            val c = mod.module.params
            val wdog = IO(new WDIO( c.Resets ))

            wdog.outputs := mod.module.io.wdog.outputs
            mod.module.io.wdog.clock := wdog.clock
            crossing.module.clock := wdog.clock
            toaxi4.module.clock := wdog.clock
            wdog
          }
          case Left(mod) =>{
            val c = mod.module.params
            val wdog = IO(new WDIO( c.Resets ))

            wdog.outputs := mod.module.io.wdog.outputs
            mod.module.io.wdog.clock := wdog.clock
            crossing.module.clock := wdog.clock
            wdog
          }
        }
      })
    }
    case None => None
  }
}

object TLWDT {
  val nextId = { var i = -1; () => { i += 1; i} }

  def attach(params: WDTParams, parameter: Parameters, pBus: PeripheryBus, iBus: InterruptBusWrapper) : (Either[TLWDT, (AXI4WDT, TLToAXI4)],TLAsyncCrossingSink) = {
    implicit val p = parameter
    if (params.useAXI4){
      val name = s"axi4wdogarray_${nextId()}"
      val crossing = LazyModule(new TLAsyncCrossingSink(AsyncQueueParams.singleton()))
      val tlaxi4 = LazyModule(new TLToAXI4())
      val wdt = LazyModule(new AXI4WDT(params, pBus.beatBytes)(p))
      wdt.suggestName(name)
      pBus.toSlave(Some(name)) {
        wdt.node :=
        AXI4Buffer () :=
        tlaxi4.node :=
        crossing.node :=
        TLAsyncCrossingSource() :=
        // toVariableWidthSlave doesn't use holdFirstDeny, which TLToAXI4() needsx
        TLFragmenter(pBus.beatBytes, pBus.blockBytes, holdFirstDeny = true) 
      }
      iBus.fromSync := IntSyncCrossingSink() := IntSyncCrossingSource(alreadyRegistered = true)  := wdt.intnode // Ibus -> Connecting to Interruptsystem
      
      (Right(wdt, tlaxi4),crossing)
    }else{
      val name = s"tlwdogarray_${nextId()}"
      val crossing = LazyModule(new TLAsyncCrossingSink(AsyncQueueParams.singleton()))
      val wdt = LazyModule(new TLWDT(params, pBus.beatBytes)(p))
      wdt.suggestName(name)
      val node = wdt.node := crossing.node 
      pBus.toVariableWidthSlave(Some(name)) { node := TLAsyncCrossingSource() }  // Pbus -> Connecting to PeripherialBus // sbus -> Systembus
      iBus.fromSync := IntSyncCrossingSink() := IntSyncCrossingSource(alreadyRegistered = true)  := wdt.intnode // Ibus -> Connecting to Interruptsystem
      
      (Left(wdt),crossing)
    }
  }
}