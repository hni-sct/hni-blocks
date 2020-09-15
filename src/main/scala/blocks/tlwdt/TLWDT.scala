// See LICENSE.SiFive for license details.

package hni.blocks.wdt

//import Chisel._
import hni.blocks.devices.watchdog._

import Chisel._
//import chisel3.util._
//import chisel3.experimental.{IntParam, BaseModule, withClock}
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.subsystem.BaseSubsystem
import freechips.rocketchip.config.{Parameters, Field, Config}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap}//, RegField
import freechips.rocketchip.tilelink._
//import freechips.rocketchip.devices.tilelink.CanHavePeripheryCLINT
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

class WDIO(Resets : Int) extends Bundle {
  val outputs = (Vec(Resets,Bool(OUTPUT)))
  val clock = Clock(INPUT)
}

trait WDTBundle
{
  val params: WDTParams
  val c = params
  //val wdog_rst = Bool(OUTPUT)

  // Wdog MUX
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
  new TLRegModule(params, _, _) with WDTModule) //with HasInterruptSources

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
  val wdt = p(WDTKey) match {
    case Some(params) => {
      if (params.useAXI4) {
        val wdt = LazyModule(new AXI4WDT(params, pbus.beatBytes)(p))
        pbus.toSlave(Some(portName)) {
          wdt.node :=
          AXI4Buffer () :=
          TLToAXI4 () :=
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

trait CanHavePeripheryWDTModuleImp extends LazyModuleImp {
  val outer: CanHavePeripheryWDT
  
  val wdt_out = outer.wdt match {
    case Some(wdt) => {
      val c = wdt.module.params
      val wdog = IO(new WDIO(  c.Resets ))
      wdog.outputs := wdt.module.io.wdog.outputs
      wdt.module.io.wdog.clock := wdog.clock
      outer.crossing.module.clock := wdog.clock
      Some(wdog)
    }
    case None => None
  }
}

class WithWDT(address: BigInt = 0x2000, regBytes: Int = 4, useAXI4: Boolean = false,
              Dogs : Int = 4, Resets : Int = 4, Ints : Int = 3, 
              Mode : hniWatchdogTimer.Modes = hniWatchdogTimer.both, countWidth : Int = 31, cmpWidth : Int = 16, PulseWidth : Int = 32,
              PRBS : Boolean = false, PRBS_Set : Set[Int] = Set(31,28), Key : Int = 0x51F15E) 
  extends Config((site, here, up) => {
  case WDTKey => Some(WDTParams(address = address, regBytes = regBytes, useAXI4 = useAXI4,
                                Dogs = Dogs, Resets = Resets, Ints = Ints, 
                                Mode = Mode, countWidth = countWidth, cmpWidth = cmpWidth, PulseWidth = PulseWidth,
                                PRBS = PRBS, PRBS_Set = PRBS_Set, Key = Key ))
})
